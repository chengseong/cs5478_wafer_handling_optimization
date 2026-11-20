import numpy as np
import toppra as ta
from toppra.constraint import LinearConstraint, DiscretizationType
from scara_kinematics import ScaraKinematics


class CartesianFrictionConeConstraint(LinearConstraint):
    """
    Exact operational-space 2-norm acceleration constraint for planar non-prehensile handling.
    Evaluates ||a_tip||_2 <= a_max along a joint-space geometric path q(s) by linearizing
    the circular Coulomb friction boundary via an m-facet circumscribed or inscribed polygon:

        n_k^T * a_tip(s) <= g_k   for k = 1, ..., m

    where:
        a_tip(s) = J(q) * q_ddot + J_dot(q, q_dot) * q_dot
                 = [ J(q) * q'(s) ] * u + [ J(q) * q''(s) + J_dot(q, q'(s)) * q'(s) ] * w
                 = alpha(s) * u + beta(s) * w

    Matching TOPP-RA's canonical linear inequality:
        a_k(s) * u + b_k(s) * w <= c_k(s)
    """

    def __init__(self, kin: ScaraKinematics, a_max: float = 2.084625, num_facets: int = 8,
                 discretization_scheme=DiscretizationType.Collocation, inscribe: bool = True):
        super().__init__()
        self.kin = kin
        self.a_max = a_max
        self.num_facets = num_facets
        self.dof = 3
        self.set_discretization_type(discretization_scheme)
        self.identical = True
        self._format_string = (
            f"    Cartesian Friction Cone: a_max={a_max:.4f} m/s^2, facets={num_facets}, inscribe={inscribe}\n"
        )

        angles = np.linspace(0, 2 * np.pi, num_facets, endpoint=False)
        self.F = np.column_stack([np.cos(angles), np.sin(angles)])

        # Scale by cos(pi / m) to guarantee the 2-norm ball is inscribed inside polygon vertices
        if inscribe:
            facet_radius = a_max * np.cos(np.pi / num_facets)
        else:
            facet_radius = a_max

        self.g = np.full(num_facets, facet_radius)

    def compute_constraint_params(self, path, gridpoints, *args, **kwargs):
        N = len(gridpoints)
        q_vec = path(gridpoints)
        qs_vec = path(gridpoints, order=1)
        qss_vec = path(gridpoints, order=2)

        a_mat = np.zeros((N, 2))
        b_mat = np.zeros((N, 2))
        c_mat = np.zeros((N, 2))

        for i in range(N):
            q = q_vec[i]
            qs = qs_vec[i]
            qss = qss_vec[i]

            J = self.kin.jacobian(q)
            J_dot = self.kin.jacobian_derivative(q, qs)

            # alpha(s) = J(q) * q'(s) multiplies u = s_ddot
            # beta(s)  = J(q) * q''(s) + J_dot(q, q') * q' multiplies w = s_dot^2
            a_mat[i] = J @ qs
            b_mat[i] = J @ qss + J_dot @ qs

        return a_mat, b_mat, c_mat, self.F, self.g, None, None

