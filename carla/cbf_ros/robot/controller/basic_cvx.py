from env.simple_env import Environment
import numpy as np
from env.config import epsilon_s, alpha_fov, radius
import cvxpy as cp
from utils import SDF_RT, polygon_SDF
import matplotlib.pyplot as plt


class BasicController:
    def __init__(self, mapinfo=None):
        self.map_info = mapinfo
        self.map = None

    @staticmethod
    def G(x):
        theta = x[2]
        return np.array([[np.cos(theta), 0], [np.sin(theta), 0], [0, 1]])

    def R(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta)],
                         [np.sin(theta), np.cos(theta)]])

    def solvecvx(self, rbt, tgt, ydot, ref, scan, mapinfo, slam_map, u_prev):
        """CVXPY solver"""
        self.map_info = mapinfo
        self.map = slam_map
        '''Visibility CBF constraint'''
        rdrx, rdry, sdf = self.visibility_gradient(rbt, tgt)
        # return None, self.fov(rbt), -sdf, 50.0
        P_visibility = np.array([[1, 0], [0, 1]])  # Quadratic term (must be positive semi-definite)
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
        safe_observation_margin = 30
        delta = cp.Variable()
        p = 1
        visibility_const = LGh @ u + dhdt + alf * (h - safe_observation_margin) >= -delta

        '''Control constraints'''
        vel_const_min_w = -1. <= u[1]
        vel_const_max_w = u[1] <= 1.
        vel_const_min_v = 0. <= u[0]
        vel_const_max_v = u[0] <= 12.

        obs_dist = 30.0
        '''DR CBF obstacle avoidance constraint'''
        top_1_obstacle = self.sample_cbf(scan, rbt)
        obs_const = True
        if top_1_obstacle is not None:
            l = 0.5
            r = 2.0
            obs_vec = top_1_obstacle - rbt[:2].T
            obs_dist = np.linalg.norm(obs_vec)
            obs_grad = (obs_vec / np.linalg.norm(obs_vec)).T
            obs_const = obs_grad.T @ self.R(rbt[2, 0]) @ np.array([[1, 0], [0, l]]) @ u + 1 * (obs_dist - r) >= 0

        objective = cp.Minimize(cp.quad_form((u - ref), P_visibility) + 2 * cp.quad_form((u - u_prev), P_visibility) + p * cp.square(delta))
        constraints = [visibility_const, obs_const, vel_const_min_w, vel_const_max_w, vel_const_min_v, vel_const_max_v]
        prob = cp.Problem(objective, constraints)
        prob.solve()
        if u.value is None:
            print('NOT OPTIMAL SOLUTION!!')
            return ref, self.fov(rbt), h, obs_dist
        return u.value, self.fov(rbt), h, obs_dist

    def sample_cbf(self, scan_w, rbt):
        if len(scan_w) == 0: return None
        distances = np.linalg.norm(scan_w - rbt[:2].T, axis=1)
        inner_radius = 0.2
        outer_radius = 70.0
        mask = (distances >= inner_radius) & (distances <= outer_radius)
        valid_indices = np.where(mask)[0]  # This gives the indices of valid distances
        if valid_indices.size > 0:
            min_index = valid_indices[np.argmin(distances[mask])]
            min_value = distances[min_index]
            return scan_w[min_index]
        else:
            return None

    def visibility_gradient(self, rbt, tgt):
        grad_est, SDF_center = self.new_fd_grad(rbt, tgt)
        return grad_est.reshape(3, ), np.hstack([-grad_est[:2].reshape(2, ), [0]]), SDF_center

    def new_fd_grad(self, rbt, tgt):
        delta = 1.0
        delta_t = 0.1
        X = np.array(
            ([[delta, 0, 0], [-delta, 0, 0], [0, delta, 0], [0, -delta, 0], [0, 0, delta_t], [0, 0, -delta_t]]))
        df_dx = (self.sdf(rbt + X[0], tgt) - self.sdf(rbt + X[1], tgt)) / (2 * delta)
        df_dy = (self.sdf(rbt + X[2], tgt) - self.sdf(rbt + X[3], tgt)) / (2 * delta)
        df_dt = (self.sdf(rbt + X[4], tgt) - self.sdf(rbt + X[5], tgt)) / (2 * delta_t)
        return np.array([df_dx, df_dy, df_dt]), self.sdf(rbt, tgt)

    def sdf(self, rbt, tgt):
        rbt_d = self.w2m(self.map_info, rbt.reshape((3, 1))).squeeze()
        rt_visible = SDF_RT(rbt_d, np.pi / 3, 40, 50, self.map)
        visible_region = (self.m2w(self.map_info, rt_visible.T)[0:2, :]).T
        return polygon_SDF(visible_region, tgt[0:2].flatten())

    def fov(self, rbt):
        rbt_d = self.w2m(self.map_info, rbt.reshape((3, 1))).squeeze()
        rt_visible = SDF_RT(rbt_d, np.pi / 3, 40, 50, self.map)
        visible_region = (self.m2w(self.map_info, rt_visible.T)[0:2, :]).T
        return visible_region

    def w2m(self, map_info, x_w):
        res = map_info.resolution
        h = map_info.height
        x_w = x_w.reshape((3, -1))
        m_w = np.array([[map_info.origin.position.x], [map_info.origin.position.y], [0]])
        scale = np.array([[1 / res, 0, 0], [0, 1 / res, 0], [0, 0, 1]])
        dRm = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        dtm = np.array([[h], [0], [np.pi / 2]])
        return dRm @ scale @ (x_w - m_w) + dtm

    def m2w(self, map_info, x_m):
        res = map_info.resolution
        h = map_info.height
        if len(x_m) == 2:
            x_m = np.vstack((x_m, np.zeros((1, len(x_m[0])))))
        x_m = x_m.reshape((3, -1))
        m_w = np.array([[map_info.origin.position.x], [map_info.origin.position.y], [0]])
        scale = np.array([[res, 0, 0], [0, res, 0], [0, 0, 1]])
        mRd = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        mtd = np.array([[-h], [0], [-np.pi / 2]])
        return scale @ mRd @ (x_m + mtd) + m_w

    @staticmethod
    def crop(u):
        u[0] = min(max(u[0], 0), 12)
        u[1] = min(max(u[1], -1.), 1.)
        return u
