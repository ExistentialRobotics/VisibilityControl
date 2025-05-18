import cvxpy as cp
import numpy as np
from world.env.config import epsilon_s, alpha_fov, radius
from world.env.simple_env import Environment
from utils.utils import map2world, world2map


class CbfController:
    def __init__(self, env: Environment):
        self.motion_model = None
        self.sensing = None
        self.env = env
        self.ref_input = None

    def c(self, x, ydot, ref, d, rdrx, rdry):
        alpha = 3
        mat = -(rdrx @ self.F(x) + rdrx @ self.G(x) @ ref + rdry @ ydot)
        return mat + alpha * self.hS(d) * np.ones(np.shape(mat))

    @staticmethod
    def F(x):
        return np.array([[0], [0], [0]])

    @staticmethod
    def G(x):
        theta = x[2]
        return np.array([[np.cos(theta), 0], [np.sin(theta), 0], [0, 1]])

    def hS(self, d):
        return -d - epsilon_s * self.min_distance(alpha_fov, radius)

    @staticmethod
    def min_distance(alpha, radius):
        return radius * np.sin(alpha) / (2 + 2 * np.sin(0.5 * alpha))

    def L_Gh(self, x, rdrx):
        return (rdrx @ self.G(x)).reshape(1, 2)

    def R(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta)],
                         [np.sin(theta), np.cos(theta)]])

    def u_weighted(self, x, rdrx, c):
        p = 1
        la = np.array([[1, 0], [0, 1 / np.sqrt(p)]])
        return -c * la @ la @ self.L_Gh(x, rdrx).T / np.linalg.norm(self.L_Gh(x, rdrx) @ la)

    def solve(self, x, y, ydot, ref):
        rdrx, rdry, signed_d = self.env.polygon_sdf_grad_lse(x, y, True)
        u = np.copy(ref).astype(np.float64)
        c = self.c(x, ydot, ref, signed_d, -rdrx, -rdry)
        u_bar = self.u_weighted(x, -rdrx, c)
        gamma = 1. if signed_d > 15 else 5.
        if c < 0:
            u[0] += gamma * u_bar[0]
            u[1] += gamma * u_bar[1]
        return self.crop(u)

    def solvecvx(self, rbt, tgt, ydot, ref, u_prev):
        """CVXPY solver"""

        '''Visibility CBF constraint'''
        rdrx, rdry, sdf = self.env.polygon_sdf_grad_lse(rbt, tgt, True)
        P_visibility = np.array([[1, 0], [0, 1]])  # Quadratic term (must be positive semi-definite)
        P_safety = np.array([[0.01, 0], [0, 1]])
        u = cp.Variable((2, 1))
        ref = ref.reshape((2, 1))
        rbt = rbt.reshape((3, 1))
        grad = rdrx.reshape((1, 3))
        grady = rdry.reshape((1, 3))
        ydot = ydot.reshape((3, 1))
        alf = 3.0
        LGh = -grad @ self.G(rbt.flatten())
        dhdt = -grady @ ydot
        h = -sdf
        safe_observation_margin = 20
        visibility_const = LGh @ u + dhdt + alf * (h - safe_observation_margin) >= 0

        '''DR CBF obstacle avoidance constraint'''
        top_1_obstacle = self.sample_cbf(rbt)
        obs_const = True
        if top_1_obstacle is not None:
            l = 0.5
            r = 5.0
            obs_vec = top_1_obstacle - rbt[:2].T
            obs_dist = np.linalg.norm(obs_vec)
            obs_grad = (obs_vec / np.linalg.norm(obs_vec)).T
            obs_const = -obs_grad.T @ self.R(rbt[2, 0]) @ np.array([[1, 0], [0, l]]) @ u + 1 * (obs_dist - r) >= 0

        '''Motion constraints'''
        vel_const_min_w = -1.75 <= u[1]
        vel_const_max_w = u[1] <= 1.75

        objective = cp.Minimize(cp.quad_form((u - ref), P_visibility))
        constraints = [visibility_const, obs_const]
        prob = cp.Problem(objective, constraints)
        prob.solve()
        if u.value is None:
            print('NOT OPTIMAL SOLUTION!!')
            return ref, top_1_obstacle
        return self.crop(u.value), top_1_obstacle

    def sample_cbf(self, rbt):
        rbt_d = world2map(self.env.map_origin, self.env.map_ratio, rbt.reshape((3, 1))).squeeze()
        # minimum vertex polygon function is called inside SDF_RT (raytracing)
        rt_visible = self.env.SDF_RT_circular(rbt_d, 100, 300)
        scan_w = (map2world(self.env.map_origin, self.env.map_ratio, rt_visible.T)[0:2, :]).T
        if len(scan_w) == 0: return None
        distances = np.linalg.norm(scan_w - rbt[:2].T, axis=1)
        inner_radius = 0.1
        outer_radius = 30.0
        mask = (distances >= inner_radius) & (distances <= outer_radius)
        valid_indices = np.where(mask)[0]  # This gives the indices of valid distances
        if valid_indices.size > 0:
            min_index = valid_indices[np.argmin(distances[mask])]
            min_value = distances[min_index]
            return scan_w[min_index]
        else:
            return None

    @staticmethod
    def crop(u):
        u[0] = min(max(u[0], 0), 50)
        u[1] = min(max(u[1], -1.75), 1.75)
        return u
