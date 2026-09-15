"""Implicit nonlinear-shell impact with internal nodal inertia and rigid frames.

Provisional lumped masses, finite-rotation frames and compliant normal contact.
Backward Euler adds numerical damping; no material/bond/failure validation.
"""

from dataclasses import asdict, dataclass
import math

import numpy as np
import torch
from scipy.optimize import minimize

from exact_joint.shell_contact import intersection_pairs


@dataclass(frozen=True)
class ShellImpactConfig:
    height_m: float = .07
    upper_mass_kg: float = .03
    lower_mass_kg: float = .02
    contact_stiffness_n_m: float = 20000.0
    contact_damping_ns_m: float = .1
    step_s: float = .0001
    duration_s: float = .015
    pet_density_kg_m3: float = 1390.0
    pla_density_kg_m3: float = 1240.0
    max_iterations: int = 180

    @property
    def step_count(self):
        """Match the integrator's 1e-12 s end tolerance, not float-division ceil."""
        return max(1, math.ceil((self.duration_s-1e-12)/self.step_s))

    def validate(self):
        if not np.isfinite(list(asdict(self).values())).all():
            raise ValueError("Dynamic parameters must be finite")
        if not 0 < self.height_m <= .2 or not .001 <= min(self.upper_mass_kg, self.lower_mass_kg):
            raise ValueError("Use a positive height up to 200 mm and positive assembly masses >=1 g")
        if max(self.upper_mass_kg, self.lower_mass_kg) > .5:
            raise ValueError("Each assumed assembly mass must be at most 500 g")
        if not 100 <= self.contact_stiffness_n_m <= 1e6 or not 0 <= self.contact_damping_ns_m <= 100:
            raise ValueError("Contact stiffness 100-1e6 N/m; damping 0-100 N s/m")
        if not 1e-6 <= self.step_s <= .0002 or not self.step_s <= self.duration_s <= .1:
            raise ValueError("Use 1-200 us steps and a duration up to 100 ms")
        if min(self.pet_density_kg_m3, self.pla_density_kg_m3) <= 0:
            raise ValueError("Densities must be positive")
        if self.max_iterations != int(self.max_iterations) or not 10 <= self.max_iterations <= 5000:
            raise ValueError("Dynamic iteration limit must be an integer 10-5000")


class ShellImpact:
    def __init__(self, shell, config=None):
        self.shell = shell
        self.config = config or ShellImpactConfig()
        self.config.validate()
        mat, cfg = shell.material, self.config
        areal_mass = (mat.pet_thickness_m*cfg.pet_density_kg_m3
                      + shell.mesh["laminate"]*mat.pla_thickness_m*cfg.pla_density_kg_m3)
        nodal_mass = np.zeros(len(shell.points))
        np.add.at(nodal_mass, shell.mesh["triangles"].ravel(),
                  np.repeat(shell.area.numpy()*areal_mass/3, 3))
        self.shell_mass_kg = float(nodal_mass.sum())
        self.rigid_masses = np.array([cfg.upper_mass_kg, cfg.lower_mass_kg]) - self.shell_mass_kg/2
        if self.rigid_masses.min() <= 0:
            raise ValueError("Assembly mass is below its assigned half of the shell mass")
        signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
        dimensions = np.array([[.028, .025, .090], [.050, .026, .090]])
        com = np.array([[0, 0, shell.height+.045], [.006, 0, -.050]])
        # Equal point masses at +/- side/(2 sqrt(3)) match a uniform box inertia.
        self.body_points = [shell._tensor(com[i]+signs*dimensions[i]/(2*np.sqrt(3))) for i in range(2)]
        self.masses = shell._tensor(np.r_[nodal_mass, np.repeat(self.rigid_masses/8, 8)])
        self.foot_points = shell._tensor([[x, y, -.0875] for x in (-.013, .037) for y in (-.013, .013)])
        self.ground_z = -.0875
        self.state = np.zeros(shell.ndof)
        self.time_s = 0.0
        self.stopped = False
        self.reason = None
        self.trace = []
        self.last_candidate = None
        self.previous_particles = self.particles(shell._tensor(self.state)).detach()
        self.velocity = torch.zeros_like(self.previous_particles)
        self.velocity[:, 2] = -np.sqrt(2*9.81*cfg.height_m)
        self.initial_energy_j = .5*float(torch.sum(self.masses[:, None]*self.velocity**2))
        self.energy_reference_z = self.previous_particles[:, 2].clone()
        # Linearized mass is used only to scale the optimizer. Kinematics,
        # inertia forces and contact are evaluated nonlinearly in the objective.
        from scipy.sparse import diags
        jacobian = self.particle_jacobian(self.state)
        self.mass_scaling = (jacobian.T@diags(np.repeat(self.masses.numpy(), 3))@jacobian).tocsc()
        if not shell.config.sparse_solver:
            self.mass_scaling = self.mass_scaling.toarray()
        self.scaling_cache = {}
        self.record()

    def particle_jacobian(self, state):
        """Local frame Jacobians avoid a dense 3N-by-DOF mass allocation."""
        from scipy.sparse import coo_matrix, vstack
        from exact_joint.nonlinear_shell import rotation_matrix
        from exact_joint.shell_sparse import position_jacobian
        shell = self.shell
        rows, columns, values = [], [], []
        for frame in range(2):
            offset = shell.frame_start+6*frame
            center = shell.frame_centers[frame]

            def positions(q):
                return (self.body_points[frame]-center)@rotation_matrix(q[3:]).T+center+shell.width*q[:3]

            jac = torch.autograd.functional.jacobian(positions, shell._tensor(state[offset:offset+6])).numpy().reshape(24, 6)
            rows.extend(np.broadcast_to((24*frame+np.arange(24))[:, None], jac.shape).ravel())
            columns.extend(np.broadcast_to(offset+np.arange(6), jac.shape).ravel())
            values.extend(jac.ravel())
        bodies = coo_matrix((values, (rows, columns)), shape=(48, shell.ndof)).tocsr()
        return vstack((position_jacobian(shell, state), bodies), format="csr")

    def particles(self, state):
        shell = self.shell
        return torch.cat([shell.positions(state)] + [shell.frame_point(state, i, self.body_points[i]) for i in range(2)])

    def gaps(self, state):
        return self.shell.frame_point(state, 1, self.foot_points)[:, 2] - self.ground_z

    def step(self):
        """One force-driven solve; reject a candidate before updating accepted state."""
        if self.stopped:
            return self.trace[-1]
        shell, cfg = self.shell, self.config
        dt = min(cfg.step_s, cfg.duration_s-self.time_s)
        predicted = self.previous_particles + dt*self.velocity
        previous_gaps = self.gaps(shell._tensor(self.state)).detach()

        def objective(value):
            state = shell._tensor(value).clone().requires_grad_(True)
            particles = self.particles(state)
            energy = sum(shell.elastic_terms(particles[:len(shell.points)]))
            energy = energy + .5/dt**2 * torch.sum(self.masses[:, None]*(particles-predicted)**2)
            energy = energy + 9.81*torch.sum(self.masses*(particles[:, 2]-self.energy_reference_z))
            gap = self.gaps(state)
            energy = energy + cfg.contact_stiffness_n_m/8*torch.sum(torch.clamp_max(gap, 0)**2)
            closing = torch.clamp_min(previous_gaps-gap, 0) * (gap < 0)
            energy = energy + cfg.contact_damping_ns_m/(8*dt)*torch.sum(closing**2)
            grad = torch.autograd.grad(energy, state)[0]
            result, derivative = float(energy.detach()), grad.detach().numpy()
            if shell.contact is not None:
                from exact_joint.shell_sparse import position_jacobian
                contact_energy, contact_gradient, _ = shell.contact.evaluate(particles[:len(shell.points)].detach().numpy())
                result += contact_energy
                derivative += position_jacobian(shell, value).T@contact_gradient
            return result, derivative

        # Same neutral geometry, masses and nominal timestep share a scaling.
        # Cache only optimization coordinates, never physical force evaluations.
        if shell.config.sparse_solver:
            from scipy.sparse import csr_matrix, diags
            from exact_joint.shell_sparse import minimize_sparse

            def inertia_contact_stiffness(state):
                jac = self.particle_jacobian(state)
                result = jac.T@diags(np.repeat(self.masses.numpy(), 3)/dt**2)@jac
                value = shell._tensor(state)
                gap = self.gaps(value).numpy()
                gap_jac = csr_matrix(torch.autograd.functional.jacobian(self.gaps, value, vectorize=True).numpy())
                weight = (gap < 0)*(cfg.contact_stiffness_n_m
                          + (previous_gaps.numpy() > gap)*cfg.contact_damping_ns_m/dt)/4
                return result+gap_jac.T@diags(weight)@gap_jac

            solution = minimize_sparse(shell, self.state, np.arange(shell.ndof), objective,
                                       extra_stiffness=inertia_contact_stiffness, max_iterations=cfg.max_iterations,
                                       gradient_tolerance=1e-6)
        else:
            key = round(dt, 14)
            if key not in self.scaling_cache:
                self.scaling_cache[key] = shell.stiffness_scaling(shell.reference_hessian()+self.mass_scaling/dt**2)
            solution = shell.minimize_scaled(objective, self.state, None, cfg.max_iterations,
                                             transform=self.scaling_cache[key])
        candidate = shell.diagnostics(solution.x)
        residual = float(np.abs(solution.jac).max())
        candidate["residual_j_per_scaled_coordinate"] = residual
        candidate["optimizer_iterations"] = int(solution.nit)
        candidate["optimizer_message"] = str(solution.message)
        self.last_candidate = candidate
        if not np.isfinite(solution.fun) or residual > 3e-5:
            self.stopped, self.reason = True, "SOLVER_RESIDUAL_LIMIT; candidate not applied"
        elif candidate["max_membrane_strain"] > shell.config.panel_strain_limit:
            self.stopped, self.reason = True, "MEMBRANE_STRAIN_GUARD; candidate not applied; NOT fracture"
        elif candidate["frame_separation_m"] < shell.height*shell.config.minimum_height_fraction:
            self.stopped, self.reason = True, "FRAME_SEPARATION_GUARD; candidate not applied"
        elif len(intersection_pairs(candidate["points"], shell.mesh["triangles"])):
            self.stopped, self.reason = True, "SURFACE_INTERSECTION_GUARD; candidate not applied"
        if self.stopped:
            return self.record()
        self.state = solution.x.copy()
        particles = self.particles(shell._tensor(self.state)).detach()
        self.velocity = (particles-self.previous_particles)/dt
        self.previous_particles = particles
        self.time_s += dt
        gap = self.gaps(shell._tensor(self.state)).detach().numpy()
        self.corner_forces = (cfg.contact_stiffness_n_m*np.maximum(-gap, 0)
                              + cfg.contact_damping_ns_m/dt*np.maximum(previous_gaps.numpy()-gap, 0)*(gap < 0))/4
        if self.time_s >= cfg.duration_s-1e-12:
            self.stopped, self.reason = True, "TIME_WINDOW_COMPLETE; survival not evaluated"
        return self.record(residual=residual, iterations=int(solution.nit))

    def record(self, residual=0.0, iterations=0):
        report = self.shell.diagnostics(self.state)
        with torch.no_grad():
            gap = self.gaps(self.shell._tensor(self.state)).numpy()
            energy = (sum(report["energy_j"].values()) + .5*float(torch.sum(self.masses[:, None]*self.velocity**2))
                      + 9.81*float(torch.sum(self.masses*(self.previous_particles[:, 2]-self.energy_reference_z)))
                      + self.config.contact_stiffness_n_m/8*np.sum(np.minimum(gap, 0)**2))
        row = {"time_after_contact_s": self.time_s, "ground_force_n": float(np.sum(getattr(self, "corner_forces", np.zeros(4)))),
               "corner_forces_n": np.asarray(getattr(self, "corner_forces", np.zeros(4))).tolist(),
               "relative_rotation_rad": report["relative_rotation_rad"].tolist(),
               "compression_fraction": report["compression_fraction"], "max_membrane_strain": report["max_membrane_strain"],
               "strain_energy_components_j": report["energy_j"], "mechanical_energy_j": float(energy),
               "frame_hinge": report["frame_hinge"],
               "energy_fraction_of_initial": float(energy/self.initial_energy_j),
               "minimum_foot_gap_m": float(gap.min()), "nonlinear_iterations": iterations,
               "residual_j_per_scaled_coordinate": residual, "stop_reason": self.reason}
        if self.shell.contact is not None:
            contact = self.shell.contact.report(report["points"])
            row["self_contact"] = contact
            row["mechanical_energy_j"] += contact["barrier_energy_j"]
            row["energy_fraction_of_initial"] = row["mechanical_energy_j"]/self.initial_energy_j
        self.trace.append(row)
        return row
