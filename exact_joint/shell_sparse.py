"""Sparse element-local Gauss-Newton assembly for the same shell energy."""

import numpy as np
import torch
from scipy.optimize import OptimizeResult
from scipy.sparse import coo_matrix, csr_matrix, diags, vstack
from scipy.sparse.linalg import spsolve


def position_jacobian(shell, state):
    from exact_joint.nonlinear_shell import rotation_matrix

    rows = (3*shell.free_nodes[:, None]+np.arange(3)).ravel().tolist()
    columns = np.arange(shell.frame_start).tolist()
    values = [shell.width]*shell.frame_start
    for frame, ids in enumerate(shell.frame_indices):
        offset = shell.frame_start+6*frame
        center = shell.frame_centers[frame]

        def positions(q):
            return (shell.rest[ids]-center)@rotation_matrix(q[3:]).T+center+shell.width*q[:3]

        jac = torch.autograd.functional.jacobian(positions, shell._tensor(state[offset:offset+6])).numpy()
        rr = np.broadcast_to((3*ids.numpy()[:, None]+np.arange(3))[:, :, None], jac.shape)
        cc = np.broadcast_to(offset+np.arange(6), jac.shape)
        rows.extend(rr.ravel()); columns.extend(cc.ravel()); values.extend(jac.ravel())
    result = coo_matrix((values, (rows, columns)), shape=(len(shell.points)*3, shell.ndof)).tocsr()
    result.eliminate_zeros()
    return result


def membrane_residual(p, inverse, weight, nu):
    gradient = torch.stack((p[1]-p[0], p[2]-p[0]), dim=1)@inverse
    strain = .5*(gradient.T@gradient-torch.eye(2, dtype=p.dtype))
    return torch.cat((weight*strain.ravel(),
                      (weight*torch.sqrt(nu/(1-nu))*torch.trace(strain)).reshape(1)))


def hinge_angle(p):
    a, b, c, d = p.unbind()
    edge = b-a
    unit = edge/torch.linalg.vector_norm(edge).clamp_min(1e-14)
    n1, n2 = torch.linalg.cross(edge, c-a), torch.linalg.cross(d-a, edge)
    n1 = n1/torch.linalg.vector_norm(n1).clamp_min(1e-18)
    n2 = n2/torch.linalg.vector_norm(n2).clamp_min(1e-18)
    return torch.atan2(torch.dot(torch.linalg.cross(n1, n2), unit), torch.dot(n1, n2))


def local_jacobian_matrix(local, nodes, row_offset, row_width, ncolumns):
    local = local.reshape(len(nodes), row_width, -1)
    rows = np.broadcast_to((row_offset+np.arange(len(nodes)*row_width)).reshape(len(nodes), row_width, 1), local.shape)
    columns = np.broadcast_to((3*nodes[:, :, None]+np.arange(3)).reshape(len(nodes), 1, -1), local.shape)
    return coo_matrix((local.ravel(), (rows.ravel(), columns.ravel())),
                      shape=(row_offset+len(nodes)*row_width, ncolumns)).tocsr()


def stiffness(shell, state, winch=None):
    """Exactly J.T@J for shell residuals; linear memory in element count."""
    points = shell.positions(shell._tensor(state)).detach()
    weight = torch.sqrt(shell.area*shell.membrane_modulus/(1+shell.nu))
    local = torch.func.vmap(torch.func.jacrev(membrane_residual, argnums=0))(
        points[shell.faces], shell.inverse_reference, weight, shell.nu).numpy()
    membrane = local_jacobian_matrix(local, shell.mesh["triangles"], 0, 5, len(points)*3)
    local = torch.func.vmap(torch.func.jacrev(hinge_angle))(points[shell.hinges]).numpy()
    angle = local_jacobian_matrix(local, shell.mesh["hinges"], 0, 1, len(points)*3)
    bending = diags(np.sqrt(shell.hinge_stiffness.numpy()))@angle
    pairs = shell.twist_pairs.numpy()
    coupling = diags(np.sqrt(shell.twist_stiffness.numpy()))@(angle[pairs[:, 0]]-angle[pairs[:, 1]])
    jacobian = vstack((membrane, bending, coupling), format="csr")@position_jacobian(shell, state)
    result = (jacobian.T@jacobian).tocsc()
    if shell.contact is not None:
        _, _, contact_hessian = shell.contact.evaluate(points.numpy(), hessian=True)
        if contact_hessian.nnz:
            mapping = position_jacobian(shell, state)
            result += mapping.T@contact_hessian@mapping
    if winch is not None:
        def lengths(value):
            return torch.linalg.vector_norm(shell.frame_point(value, 0, shell.cable_top)
                                            - shell.frame_point(value, 1, shell.cable_bottom), dim=1)
        q = shell._tensor(state)
        extension = lengths(q).numpy()-np.asarray(winch.rest_lengths_m)
        k = (np.asarray(winch.active)*(extension >= -1e-12)
             * (extension < winch.force_cap_n/winch.stiffness_n_per_m))*winch.stiffness_n_per_m
        jac = csr_matrix(torch.autograd.functional.jacobian(lengths, q, vectorize=True).numpy())
        result += jac.T@diags(k)@jac
    return result


def minimize_sparse(shell, state, active, objective, winch=None, extra_stiffness=None, max_iterations=None,
                    gradient_tolerance=1e-7):
    """Damped sparse Gauss-Newton descent; actual energy/residual acceptance."""
    x = state[active].copy()
    energy, gradient = objective(x)
    message = "Iteration limit"
    iterations = 0
    for iterations in range(1, (max_iterations or shell.config.max_iterations)+1):
        if np.max(np.abs(gradient)) < gradient_tolerance:
            message = "Force residual converged"
            break
        full = state.copy(); full[active] = x
        matrix = stiffness(shell, full, winch)
        if extra_stiffness is not None:
            matrix += extra_stiffness(full)
        matrix = matrix[active][:, active].tocsc()
        ridge = max(float(matrix.diagonal().max())*1e-12, 1e-12)
        direction = spsolve(matrix+diags(np.full(len(x), ridge)), -gradient)
        if not np.isfinite(direction).all() or gradient@direction >= 0:
            message = "No finite descent direction"
            break
        # Bound an optimizer increment, not the final physical travel.
        step = min(1.0, .02/max(float(np.max(np.abs(direction))), 1e-30))
        for backtrack in range(35):
            next_x = x+step*direction
            if shell.contact is not None:
                next_full = state.copy(); next_full[active] = next_x
                if not shell.contact.shell_step_is_feasible(shell, full, next_full):
                    step *= .5
                    continue
            next_energy, next_gradient = objective(next_x)
            if np.isfinite(next_energy) and next_energy <= energy+1e-4*step*(gradient@direction):
                x, energy, gradient = next_x, next_energy, next_gradient
                break
            step *= .5
        else:
            message = "Line search stalled"
            break
    return OptimizeResult(x=x, fun=energy, jac=gradient, nit=iterations,
                          success=bool(np.max(np.abs(gradient)) < 1e-5), message=message)
