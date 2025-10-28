import numpy as np
import cvxpy as cp
import time
from utils import SDF_RT
from env.config import *
from copy import copy


class DRCController:
    def __init__(self):
        # from configs
        self.dt = dt
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_inc = angle_inc
        self.robot_radius = 0.31
        self.wasserstein_r = 0.004
        self.epsilon = 0.1
        self.alpha = 0.25  # CBF hyperparameter
        self.use_qp_bound_constraints = False


    def G(self, state):
        _, _, theta = state
        return np.array([[self.dt * np.cos(theta), 0], [self.dt * np.sin(theta), 0], [0, dt]])


    def f(self, x, u):
        y = self.G(x) @ u
        return y


    def clip_opt_u(self, u):
        u[0] = np.clip(u[0], v_lb, v_ub)
        while np.abs(u[1]) > 2 * np.pi:
            if u[1] < -2 * np.pi:
                u[1] += 2 * np.pi
            elif u[1] > 2 * np.pi:
                u[1] -= 2 * np.pi
        # u[1] = (u[1] + np.pi) % (2 * np.pi) - np.pi
        u[1] = np.clip(u[1], omega_lb, omega_ub)
        return u

    def fwd_fd(self, rbt, tgt, perturb, fov, map_info):
        """Forward finite difference method for computing gradient of SDF
        Args:
            rbt: Robot state
            tgt: Target state
            perturb: Whether to perturb robot or target state; i.e. dh/dx vs dh/dt
            fov: Robot's FoV; only applicable for perturb="tgt" as robot state is not perturbed and FoV is static
        """

        delta = 0.1
        delta_t = 0.01
        X = np.array(
            ([[delta, 0, 0], [-delta, 0, 0], [0, delta, 0], [0, -delta, 0], [0, 0, delta_t], [0, 0, -delta_t]]))
        if perturb == "rbt":
            df_dx = (-self.sdf_rt(rbt + X[0], tgt, map_info) - -self.sdf(fov, tgt)) / (delta)
            df_dy = (-self.sdf_rt(rbt + X[2], tgt, map_info) - -self.sdf(fov, tgt)) / (delta)
            df_dt = (-self.sdf_rt(rbt + X[4], tgt, map_info) - -self.sdf(fov, tgt)) / (delta_t)
        elif perturb == "tgt": # only for dh/dt term; since robot is not perturbed, dont need raytracing as FoV is static
            # -ve signs intentional for SDF convention:
            # want negative SDF for target outside FoV, positive SDF for target inside FoV
            df_dx = (-self.sdf(fov, tgt + X[0]) - -self.sdf(fov, tgt + X[1])) / (2 * delta)
            df_dy = (-self.sdf(fov, tgt + X[2]) - -self.sdf(fov, tgt + X[3])) / (2 * delta)
            df_dt = (-self.sdf(fov, tgt + X[4]) - -self.sdf(fov, tgt + X[5])) / (2 * delta_t)
        else:
            raise ValueError("Invalid argument given to perturb: only rbt, tgt accepted")

        return np.array([df_dx, df_dy, df_dt])


    def new_fd_grad(self, rbt, tgt, perturb, fov, map_info):
        """Central finite difference method for computing gradient of SDF
        Args:
            rbt: Robot state
            tgt: Target state
            perturb: Whether to perturb robot or target state; i.e. dh/dx vs dh/dt
            fov: Robot's FoV; only applicable for perturb="tgt" as robot state is not perturbed and FoV is static
        """

        delta = 0.1
        delta_t = 0.01
        X = np.array(
            ([[delta, 0, 0], [-delta, 0, 0], [0, delta, 0], [0, -delta, 0], [0, 0, delta_t], [0, 0, -delta_t]]))
        if perturb == "rbt":
            df_dx = (-self.sdf_rt(rbt + X[0], tgt, map_info) - -self.sdf_rt(rbt + X[1], tgt, map_info)) / (2 * delta)
            df_dy = (-self.sdf_rt(rbt + X[2], tgt, map_info) - -self.sdf_rt(rbt + X[3], tgt, map_info)) / (2 * delta)
            df_dt = (-self.sdf_rt(rbt + X[4], tgt, map_info) - -self.sdf_rt(rbt + X[5], tgt, map_info)) / (2 * delta_t)
        elif perturb == "tgt": # only for dh/dt term; since robot is not perturbed, dont need raytracing as FoV is static
            # -ve signs intentional for SDF convention:
            # want negative SDF for target outside FoV, positive SDF for target inside FoV
            df_dx = (-self.sdf(fov, tgt + X[0]) - -self.sdf(fov, tgt + X[1])) / (2 * delta)
            df_dy = (-self.sdf(fov, tgt + X[2]) - -self.sdf(fov, tgt + X[3])) / (2 * delta)
            df_dt = (-self.sdf(fov, tgt + X[4]) - -self.sdf(fov, tgt + X[5])) / (2 * delta_t)
        else:
            raise ValueError("Invalid argument given to perturb: only rbt, tgt accepted")

        return np.array([df_dx, df_dy, df_dt])


    def w2m(self, map_info, x_w):
        # formerly display2map
        res = map_info['resolution']
        h = map_info['height']
        x_w = x_w.reshape((3, -1))
        m_w = np.array([[map_info['origin'][0]], [map_info['origin'][1]], [0]])
        scale = np.array([[1 / res, 0, 0], [0, 1 / res, 0], [0, 0, 1]])
        dRm = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        dtm = np.array([[h], [0], [np.pi / 2]])
        return dRm @ scale @ (x_w - m_w) + dtm

    def m2w(self, map_info, x_m):
        # formerly map2display
        res = map_info['resolution']
        h = map_info['height']
        if len(x_m) == 2: x_m = np.array([[1,0],[0,1],[0,0]]) @ x_m
        m_w = np.array([[map_info['origin'][0]], [map_info['origin'][1]], [0]])
        scale = np.array([[res, 0, 0], [0, res, 0], [0, 0, 1]])
        mRd = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        mtd = np.array([[-h], [0], [-np.pi / 2]])
        return scale @ mRd @ (x_m + mtd) + m_w



    def sdf_rt(self, rbt, tgt, map_info):
        rbt_d = self.w2m(map_info, rbt.reshape((3, 1))).squeeze()
        # minimum vertex polygon function is called inside SDF_RT (raytracing)
        rt_visible = SDF_RT(rbt_d, np.pi/3, 10, 50, map_info['map'])
        visible_region = (self.m2w(map_info, rt_visible.T)[0:2, :]).T
        return self.sdf(visible_region, tgt[0:2])


    def sdf(self, polygon, point):
        """Compute the Signed Distance Function for a point from a polygon. Serves as the barrier function for CBF.
        Convention: polygon defines unsafe set; sdf is +ve outside polygon, and negative inside
        since we want to stay outside the unsafe set (i.e. stay inside the safe set).
        """
        point = point[:2] # point will always be a pose (x,y,theta)
        # closed polygon requested
        N = len(polygon) - 1
        e = polygon[1:] - polygon[:-1]
        v = point - polygon[:-1]
        pq = v - e * np.clip((v[:, 0] * e[:, 0] + v[:, 1] * e[:, 1]) /
                             (e[:, 0] * e[:, 0] + e[:, 1] * e[:, 1]), 0, 1).reshape(N, -1)
        d = np.min(pq[:, 0] * pq[:, 0] + pq[:, 1] * pq[:, 1])
        wn = 0
        for i in range(N):
            i2 = int(np.mod(i + 1, N))
            cond1 = 0 <= v[i, 1]
            cond2 = 0 > v[i2, 1]
            val3 = np.cross(e[i], v[i])
            wn += 1 if cond1 and cond2 and val3 > 0 else 0
            wn -= 1 if ~cond1 and ~cond2 and val3 < 0 else 0
        sign = 1 if wn == 0 else -1

        if make_rounded:
            return (np.sqrt(d) * sign) + 1
        else:
            return (np.sqrt(d) * sign)


    def g_fov(self, rbt_state, tgt_state, tgt_vel, u, fov, map_info):
        val1 = self.new_fd_grad(rbt_state, tgt_state, "rbt", fov, map_info)# @ self.G(rbt_state)
        val2 = alpha_fov * -self.sdf(fov, tgt_state[0:2])
        # val3 = new_fd_grad(rbt_state, tgt_state, "tgt") @ tgt_vel
        # note: use @ G @ u instead of @f since f uses x+G(x)u
        print("FoV components: ", "Gradient term: ", val1, "RHS (ah): ", val2)

        # polygon_sdf has a -ve sign for visibility CBF since safe set definition is flipped
        # i.e. inside agent FoV, SDF should be >=0; flip sign on target SDF so this is achieved
        return (self.new_fd_grad(rbt_state, tgt_state, "rbt", fov, map_info) @ self.G(rbt_state) @ u) + (alpha_fov * -self.sdf(fov, tgt_state[0:2])) \
            # + self.new_fd_grad(rbt_state, tgt_state, "tgt", fov, map_info) @ tgt_vel  # dh/dt


    def get_visibility_cbf_samples(self, fov, rbt_pose, tgt_pose, tgt_vel, map_info):
        """Get samples for sdf, gradient wrt agent state, and time
        for sampled CBF method"""
        h_samples = self.sdf(fov, tgt_pose)
        grad_h_samples = self.new_fd_grad(rbt_pose ,tgt_pose, "rbt", fov, map_info)
        dh_dt_samples = self.new_fd_grad(rbt_pose ,tgt_pose, "tgt", fov, map_info) @ tgt_vel
        return h_samples, grad_h_samples, dh_dt_samples


    def get_drc_samples(self, scan, n_samples):
        """Calculates n_samples of h, grad_x(h), dh/dt from lidar scan.
        Use header info from robot lidar from /scan to map angle increments in each scan"""

        # get n closest points from lidar scan
        scan = np.where(scan < self.robot_radius, np.inf, scan)
        min_ix = np.argsort(scan)[:n_samples]
        h_samples = scan[min_ix]
        obs_angles = angle_min + angle_inc * min_ix
        # compute direction vectors
        directions = np.stack([h_samples * np.cos(obs_angles), h_samples * np.sin(obs_angles)])
        # gradient is just the normalized direction vector i.e. the normalized obstacle coordinate in lidar frame
        grad_h_samples = directions.T[None, :] / np.linalg.norm(directions.T, axis=1)[:, None]
        dh_dt_samples = np.zeros_like(h_samples)

        return h_samples[:, None], grad_h_samples[0], dh_dt_samples[:, None]


    def solve_drc(self, rbt, tgt, tgt_vel, ref, h_samples, h_grad_samples, dh_dt_samples, fov, map_info, use_sampled_cbf_visibility, 
        h_samples_vis=None, h_grad_samples_vis=None, dh_dt_samples_vis=None):
        """Sets up and solves constrained optimization problem for DRC controller as per
        https://arxiv.org/pdf/2405.18251 eqn. 18
        Outputs optimal control for current step"""

        # initialize the optimization variables
        N = len(h_samples)
        uu = cp.Variable(2)
        beta = cp.Variable(N)
        s = cp.Variable(1)
        d = cp.Variable() # Visibility CBF slack
        M = 1

        robot_radius = 0.5
        wasserstein_r = 0.004
        epsilon = 0.1
        alpha = 1.0  # CBF hyperparameter

        # Compute q samples, a vector of dimension 5; last 0 is theta component of gradient
        q_samples = [np.hstack([dh_dt_samples[i], h_samples[i] - robot_radius, h_grad_samples[i], 0]) for i in
                     range(N)]

        visibility_cbf_constraint = []

        if use_sampled_cbf_visibility:
            # add (single) cbf sample to q_samples
            visibility_samples = np.hstack([dh_dt_samples_vis, h_samples_vis, h_grad_samples_vis])
            q_samples.append(visibility_samples)
        # else:
            # visibility_cbf_constraint.append(self.g_fov(rbt, tgt, tgt_vel, uu, fov, map_info) >= -d)


        Fu = self.f(rbt, uu)

        alpha_vector = np.array([alpha])
        alpha_vector = cp.reshape(alpha_vector, (1, 1))
        one_vector = cp.reshape(np.array([1]), (1, 1))

        Fu = cp.reshape(Fu, (3, 1))

        stacked_vector = cp.vstack([one_vector, alpha_vector, Fu])  # Stacks them vertically to make a 5x1 vector.
        ones_vector = np.ones((5, 1))

        # Modify the constraint using the augmented control vectors
        dro_cbf_constraints = [
            wasserstein_r * cp.abs(stacked_vector) / epsilon <= (
                    s - (1 / N) * cp.sum(beta) / epsilon) * ones_vector,
            beta >= 0
        ]
        dro_cbf_constraints += [beta[i] >= s - stacked_vector.T @ q_samples[i] for i in range(N)]

        # bounds on control output; switched off and applied as a post-processing step
        bound_constraints = [
            # cp.abs(uu[0]) <= v_ub,
            # cp.abs(uu[1]) <= omega_ub,
            # d >= 0
        ]

        # 1st term is minimizing deviation from reference; 2nd is for visibility CBF slack
        obj = cp.Minimize(cp.norm(uu - ref) + M * d ** 2)
        # print("Warning: Obstacle CBF deactivated")
        print("Warning: Visibility CBF deactivated")
        constraints = dro_cbf_constraints + bound_constraints #+ visibility_cbf_constraint

        prob = cp.Problem(obj, constraints)

        start_time = time.time()

        # solving problem
        # prob.solve(solver='SCS', verbose=False, max_iters=140000, eps = 1e-4)
        prob.solve(verbose=False)
        solver_time = time.time() - start_time

        # Check if the problem was solved successfully
        if uu.value is None:
            print("-------------------------- Infeasible solution-------------------------- ")

        if prob.status != 'optimal':
            solve_fail = True
            print("-------------------------- SOLVER NOT OPTIMAL -------------------------")
            print("[In solver] solver status: ", prob.status)

            return np.array([0.0, 0.0])
        else:
            solve_fail = False

        # apply bound constraint on optimal control as a post-processing step
        u_clipped = self.clip_opt_u(copy(uu.value[0:2]))
        print("Final original control: ", uu.value[0:2], "Clipped control: ", u_clipped, "Visibility slack: ", d.value)
        print("Distance to nearest obstacle: ", h_samples - robot_radius)
        for constraint in constraints:
            print("Constraint satisfied: ", constraint.value(), ", ", "Constraint dual value: ", constraint.dual_value)

        return u_clipped
