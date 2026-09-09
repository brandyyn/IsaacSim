"""Exploratory two-body impact dynamics driven by the knee's linearized FEM.

This is a reduced structural model, not full transient solid FEM. All joint mass
is included in the two assumed rigid assemblies; interior elastic modes, material
failure and geometric nonlinearity are omitted. Stop at the small-strain guard.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np
from scipy.linalg import block_diag
from scipy.spatial.transform import Rotation

from exact_joint.mechanics import CableFem


def skew(vector: np.ndarray) -> np.ndarray:
    """Return the cross-product matrix for a three-vector."""
    x, y, z = vector
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def rigid_jacobian(offset: np.ndarray) -> np.ndarray:
    """Map small cap translations/rotations to point displacement."""
    return np.column_stack((np.eye(3), -skew(offset)))


def body_mass(mass: float, center: np.ndarray, dimensions: np.ndarray) -> np.ndarray:
    """Return spatial mass at the cap for an assumed uniform box at its COM."""
    jacobian = rigid_jacobian(center)
    inertia = mass * (np.sum(dimensions**2) - dimensions**2) / 12
    result = mass * jacobian.T @ jacobian
    result[3:, 3:] += np.diag(inertia)
    return result


@dataclass(frozen=True)
class ImpactConfig:
    """SI example parameters; contact and mass properties are not measured."""

    height_m: float = 0.070
    upper_mass_kg: float = 0.030
    lower_mass_kg: float = 0.020
    gravity_m_s2: float = 9.81
    contact_stiffness_n_m: float = 20000.0
    contact_damping_ns_m: float = 0.1
    stiffness_damping_s: float = 0.00001
    step_s: float = 0.000005
    duration_s: float = 0.030

    def validate(self) -> None:
        if not np.isfinite(list(asdict(self).values())).all():
            raise ValueError("Impact parameters must be finite SI values")
        if not 0 < self.height_m <= .2:
            raise ValueError("Use a positive drop height up to 200 mm")
        if not .001 <= min(self.upper_mass_kg, self.lower_mass_kg) <= max(
            self.upper_mass_kg, self.lower_mass_kg
        ) <= .5:
            raise ValueError("Each assumed assembly mass must be 1-500 g")
        if not 100 <= self.contact_stiffness_n_m <= 1e6:
            raise ValueError("Total contact stiffness must be 100-1,000,000 N/m")
        if not 0 <= self.contact_damping_ns_m <= 100 or not 0 <= self.stiffness_damping_s <= .001:
            raise ValueError("Contact damping must be 0-100 N s/m; stiffness damping 0-0.001 s")
        if not 0 < self.step_s <= 1e-5 or not self.step_s <= self.duration_s <= .1:
            raise ValueError("Use a step up to 10 us and an impact duration up to 0.1 s")
        if not 0 < self.gravity_m_s2 <= 20:
            raise ValueError("Gravity must be positive and at most 20 m/s^2")


class FemImpact:
    """Integrate 12 small-motion coordinates with unilateral compliant contacts.

Coordinates are upper/lower roof translations and rotation vectors. Common
ballistic motion is handled before contact. Relative deformation uses the
linearized static FEM constraint modes; no angle trajectory is prescribed.
"""

    def __init__(self, fem: CableFem, config: ImpactConfig | None = None) -> None:
        self.fem = fem
        self.config = config or ImpactConfig()
        self.config.validate()
        height = fem.source["height_m"]
        self.relative = np.zeros((6, 12))
        self.relative[:3, :3] = -np.eye(3)
        self.relative[:3, 3:6] = skew(np.array([0, 0, -height]))
        self.relative[:3, 6:9] = np.eye(3)
        self.relative[3:, 3:6] = -np.eye(3)
        self.relative[3:, 9:12] = np.eye(3)
        boundary = np.concatenate([rigid_jacobian(point) for point in fem.points[fem.bottom_nodes]])
        self.modes = fem.basis @ boundary
        self.knee_stiffness = boundary.T @ fem.kcap @ boundary
        self.stiffness = self.relative.T @ self.knee_stiffness @ self.relative
        self.damping = self.config.stiffness_damping_s * self.stiffness
        self.strain_modes = np.einsum("egik,kj->egij", fem.strain_basis, boundary, optimize=True)
        # Explicit provisional inertial approximations; these include the knee
        # mass, so no separate continuum mass is added or double counted.
        self.upper_center = np.array([0, 0, .045])
        self.lower_center = np.array([.006, 0, -.050])
        self.upper_dimensions = np.array([.028, .025, .090])
        self.lower_dimensions = np.array([.050, .026, .090])
        self.mass = block_diag(
            body_mass(self.config.upper_mass_kg, self.upper_center, self.upper_dimensions),
            body_mass(self.config.lower_mass_kg, self.lower_center, self.lower_dimensions),
        )
        self.gravity = np.concatenate([
            rigid_jacobian(center).T @ np.array([0, 0, -mass * self.config.gravity_m_s2])
            for center, mass in ((self.upper_center, self.config.upper_mass_kg),
                                 (self.lower_center, self.config.lower_mass_kg))
        ])
        # Exact rendered sole: center X=12 mm, length=50 mm, width=26 mm,
        # sole bottom 87.5 mm below the lower roof. Four frictionless corners.
        self.contact_points = np.array([[x, y, -.0875] for x in (-.013, .037) for y in (-.013, .013)])
        self.contact_jacobian = np.zeros((4, 12))
        self.contact_jacobian[:, 6:] = np.stack([rigid_jacobian(p)[2] for p in self.contact_points])
        self.reset()

    def reset(self) -> None:
        """Initialize at first contact with the true ballistic incident velocity."""
        self.time_s = 0.0
        self.position = np.zeros(12)
        self.velocity = np.zeros(12)
        self.velocity[[2, 8]] = -np.sqrt(2 * self.config.gravity_m_s2 * self.config.height_m)
        forces = self.contact_forces(self.position, self.velocity)
        self.acceleration = np.linalg.solve(self.mass, self.gravity + self.contact_jacobian.T @ forces)
        self.stopped = False
        self.stop_reason = None
        self.trace = []
        self.dissipated_j = 0.0
        self.initial_energy_j = float(.5 * self.velocity @ self.mass @ self.velocity)
        self.record()

    def contact_forces(self, position: np.ndarray, velocity: np.ndarray) -> np.ndarray:
        """Nonnegative spring force plus compression-only dashpot force."""
        gap = self.contact_jacobian @ position
        speed = self.contact_jacobian @ velocity
        return (self.config.contact_stiffness_n_m * np.maximum(-gap, 0)
                + self.config.contact_damping_ns_m * np.maximum(-speed, 0) * (gap <= 0)) / 4

    def strain(self, position: np.ndarray) -> np.ndarray:
        """Recover engineering strain from the same reduced FEM coordinates."""
        return np.einsum("egij,j->egi", self.strain_modes, self.relative @ position, optimize=True)

    @staticmethod
    def peak_principal(strain: np.ndarray) -> float:
        """Return the maximum absolute principal strain over all Gauss points."""
        tensor = np.zeros((*strain.shape[:2], 3, 3))
        for axis in range(3):
            tensor[:, :, axis, axis] = strain[:, :, axis]
        for (a, b), index in (((0, 1), 3), ((1, 2), 4), ((0, 2), 5)):
            tensor[:, :, a, b] = tensor[:, :, b, a] = strain[:, :, index] / 2
        return float(np.abs(np.linalg.eigvalsh(tensor)).max())

    def guard_ratio(self, position: np.ndarray) -> float:
        """Model-use limits, not material-failure thresholds."""
        return max(self.peak_principal(self.strain(position)) / .01,
                   np.linalg.norm((self.relative @ position)[3:]) / np.deg2rad(2),
                   np.linalg.norm(position[3:6]) / np.deg2rad(2),
                   np.linalg.norm(position[9:12]) / np.deg2rad(2))

    def von_mises(self, strain: np.ndarray) -> np.ndarray:
        """Recover per-element peak von Mises stress over the four Gauss points."""
        stress = np.einsum("eij,egj->egi", self.fem.dmat, strain, optimize=True)
        xx, yy, zz, xy, yz, xz = np.moveaxis(stress, -1, 0)
        return np.sqrt(.5 * ((xx-yy)**2 + (yy-zz)**2 + (zz-xx)**2)
                       + 3 * (xy*xy+yz*yz+xz*xz)).max(axis=1)

    def newmark_step(self, dt: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Average-acceleration Newmark with an iterated contact active set."""
        predicted_position = self.position + dt * self.velocity + .25 * dt**2 * self.acceleration
        predicted_velocity = self.velocity + .5 * dt * self.acceleration
        trial_position, trial_velocity = predicted_position.copy(), predicted_velocity.copy()
        jacobian = self.contact_jacobian
        for _ in range(30):
            active = jacobian @ trial_position <= 0
            closing = active & (jacobian @ trial_velocity < 0)
            contact_k = jacobian.T @ (active[:, None] * jacobian) * self.config.contact_stiffness_n_m / 4
            contact_c = jacobian.T @ (closing[:, None] * jacobian) * self.config.contact_damping_ns_m / 4
            stiffness, damping = self.stiffness + contact_k, self.damping + contact_c
            effective = self.mass + .5 * dt * damping + .25 * dt**2 * stiffness
            acceleration = np.linalg.solve(
                effective, self.gravity - damping @ predicted_velocity - stiffness @ predicted_position
            )
            position = predicted_position + .25 * dt**2 * acceleration
            velocity = predicted_velocity + .5 * dt * acceleration
            next_active = jacobian @ position <= 0
            next_closing = next_active & (jacobian @ velocity < 0)
            if np.array_equal(active, next_active) and np.array_equal(closing, next_closing):
                return position, velocity, acceleration
            trial_position, trial_velocity = position, velocity
        raise RuntimeError("Contact active set did not converge; no state was applied")

    def advance(self) -> dict:
        """Advance one physical step; bracket and stop at the validity boundary."""
        if self.stopped:
            return self.trace[-1]
        dt = min(self.config.step_s, self.config.duration_s - self.time_s)
        position, velocity, acceleration = self.newmark_step(dt)
        if self.guard_ratio(position) >= 1:
            low, high = 0.0, dt
            for _ in range(20):
                middle = (low + high) / 2
                candidate = self.newmark_step(middle)
                if self.guard_ratio(candidate[0]) < 1:
                    low = middle
                else:
                    high = middle
            dt = low
            position, velocity, acceleration = self.newmark_step(dt)
            self.stopped = True
            self.stop_reason = "SMALL_DEFORMATION_LIMIT_REACHED; later impact response NOT computed"
        midpoint_v = (self.velocity + velocity) / 2
        midpoint_x = (self.position + position) / 2
        speed = self.contact_jacobian @ midpoint_v
        contact_power = self.config.contact_damping_ns_m / 4 * np.sum(
            np.minimum(speed, 0)**2 * (self.contact_jacobian @ midpoint_x <= 0)
        )
        self.dissipated_j += dt * float(midpoint_v @ self.damping @ midpoint_v + contact_power)
        self.position, self.velocity, self.acceleration = position, velocity, acceleration
        self.time_s += dt
        if self.time_s >= self.config.duration_s - 1e-12 and not self.stopped:
            self.stopped = True
            self.stop_reason = "REQUESTED_TIME_WINDOW_COMPLETE; no failure/survival model"
        return self.record()

    def record(self) -> dict:
        """Record computed forces, bending, energy and dynamic balance in SI."""
        q = self.relative @ self.position
        forces = self.contact_forces(self.position, self.velocity)
        gap = self.contact_jacobian @ self.position
        knee_wrench = self.knee_stiffness @ q
        energy = float(.5 * self.velocity @ self.mass @ self.velocity
                       + .5 * self.position @ self.stiffness @ self.position
                       + self.config.contact_stiffness_n_m / 8 * np.sum(np.minimum(gap, 0)**2)
                       - self.gravity @ self.position)
        residual = (self.mass @ self.acceleration + self.damping @ self.velocity
                    + self.stiffness @ self.position - self.gravity - self.contact_jacobian.T @ forces)
        strain = self.strain(self.position)
        vm = self.von_mises(strain)
        row = {
            "time_after_contact_s": self.time_s, "position_m_rad": self.position.tolist(),
            "velocity_m_s_rad_s": self.velocity.tolist(), "knee_q_m_rad": q.tolist(),
            "ground_force_n": float(forces.sum()), "corner_forces_n": forces.tolist(),
            "contact_moment_about_lower_roof_nm": np.cross(
                self.contact_points, np.column_stack((np.zeros((4, 2)), forces))
            ).sum(axis=0).tolist(),
            "elastic_knee_wrench_n_nm": knee_wrench.tolist(),
            "max_principal_strain": self.peak_principal(strain),
            "pet_peak_von_mises_pa": float(vm[self.fem.ids == 0].max()),
            "pla_peak_von_mises_pa": float(vm[self.fem.ids == 1].max()),
            "energy_plus_dissipation_j": energy + self.dissipated_j,
            "relative_energy_error": (energy + self.dissipated_j - self.initial_energy_j) / self.initial_energy_j,
            "dynamic_residual_max_n_nm": float(np.abs(residual).max()),
            "minimum_contact_gap_m": float(gap.min()), "stop_reason": self.stop_reason,
        }
        self.trace.append(row)
        return row

    def recover(self) -> dict:
        """Return FEM displacement/stress for the scene; not a static cable solve."""
        q = self.relative @ self.position
        displacement = (self.modes @ q).reshape(-1, 3)
        strain = self.strain(self.position)
        vm = self.von_mises(strain)
        # Interior recovery and stress use the same linear constraint modes.
        # The exact rigid-roof render is supplied separately by ImpactPreview.
        stats = {
            "compression_m": float(q[2]), "bend_xy_deg": np.rad2deg(q[3:5]).tolist(),
            "twist_deg": float(np.rad2deg(q[5])), "pet_peak_pa": float(vm[self.fem.ids == 0].max()),
            "pla_peak_pa": float(vm[self.fem.ids == 1].max()),
            "max_principal_strain": self.peak_principal(strain),
            "scope": "FEM-derived reduced impact dynamics; unconverged mesh; no failure or local inertial modes",
        }
        rotation = Rotation.from_rotvec(q[3:]).as_matrix()
        return {"q": q, "rotation": rotation, "displacements": displacement,
                "points": self.fem.points + displacement, "von_mises": vm, "stats": stats,
                "tensions": np.zeros(12), "bottom_anchors": self.fem.bottom_anchors @ rotation.T + q[:3]}

    def report(self) -> dict:
        """Export trace and all assumptions, without a fabricated survival verdict."""
        return {"config": asdict(self.config), "source_sha256": self.fem.source["sha256"],
                "fem_config": asdict(self.fem.config), "knee_stiffness_si": self.knee_stiffness.tolist(),
                "mass_matrix_si": self.mass.tolist(), "contact_points_m": self.contact_points.tolist(),
                "upper_com_m": self.upper_center.tolist(), "lower_com_m": self.lower_center.tolist(),
                "upper_inertia_box_dimensions_m": self.upper_dimensions.tolist(),
                "lower_inertia_box_dimensions_m": self.lower_dimensions.tolist(),
                "initial_incident_energy_j": self.initial_energy_j, "trace": self.trace,
                "stop_reason": self.stop_reason, "survives": None,
                "method": "12-DOF reduced FEM stiffness; assumed lumped rigid-assembly inertia; Newmark beta=0.25 gamma=0.5",
                "limitations": ["Unconverged legacy PET mesh and reference-normal refinement defect",
                                "Omitted interior elastic modes and stress-wave propagation",
                                "Linearized rotations and strain; stop at 1% strain or 2 degrees",
                                "Assumed mass/inertia, contact compliance and damping; not calibrated",
                                "No friction, cable preload, plasticity, self-contact, delamination or fracture"]}
