import cvxpy as cp
import numpy as np
from copy import copy
from utils.utils import SDF_RT, map2display, display2map, polygon_SDF
from env.simple_env import Environment
from robot.controller.mpc.agent import Diff_Drive_Euler
from env.config import *
from env.map.map import Map

_global_map = Map().map
map_origin = np.array([[len(_global_map) // 2], [len(_global_map[0]) // 2], [np.pi / 2]])
map_ratio = 2

walls = [(-200, -200), (200, -200), (200, 200), (-200, 200)]
e = Environment(agent=Diff_Drive_Euler(dt), horizon=horizon, tau=tau, psi=psi, radius=radius, epsilon_s=epsilon_s,
                obs=[], wls=walls)
e.reset()
tgt_future_traj, _ = e.target_future(horizon)
tgt_future_traj = tgt_future_traj.T

def new_fd_grad(rbt, tgt):
    delta = 0.1
    delta_t = 0.01
    X = np.array(
        ([[delta, 0, 0], [-delta, 0, 0], [0, delta, 0], [0, -delta, 0], [0, 0, delta_t], [0, 0, -delta_t]]))
    df_dx = (sdf(rbt + X[0], tgt) - sdf(rbt + X[1], tgt)) / (2 * delta)
    df_dy = (sdf(rbt + X[2], tgt) - sdf(rbt + X[3], tgt)) / (2 * delta)
    df_dt = (sdf(rbt + X[4], tgt) - sdf(rbt + X[5], tgt)) / (2 * delta_t)
    return np.array([df_dx, df_dy, df_dt])

def rdry(rbt, tgt):
    delta = 0.1
    Z = np.array([[delta * np.cos(2 * np.pi * i / 4),
                   delta * np.sin(2 * np.pi * i / 4)] for i in range(1, 4 + 1)])
    W = np.zeros((Z.shape[0], 1))
    for index, dy in enumerate(Z):
        W[index, 0] = sdf(rbt, tgt[0:2] + dy) - sdf(rbt, tgt)
    grad_est_y = -np.linalg.inv(Z.T @ Z) @ Z.T @ W
    res = np.hstack([grad_est_y.reshape(2, ) + 1e-10, [0]])
    return res

def obstacle_fd_grad(rbt,obstacle):
    '''sign convention for sdf flipped in polygon_SDF for fov sdf'''
    delta = 0.1
    delta_t = 0.01
    X = np.array(
        ([[delta, 0, 0], [-delta, 0, 0], [0, delta, 0], [0, -delta, 0], [0, 0, delta_t], [0, 0, -delta_t]]))
    df_dx = (-polygon_SDF(obstacle, (rbt + X[0])[:2]) + polygon_SDF(obstacle, (rbt + X[1])[:2])) / (2 * delta)
    df_dy = (-polygon_SDF(obstacle, (rbt + X[2])[:2]) + polygon_SDF(obstacle, (rbt + X[3])[:2])) / (2 * delta)
    df_dt = (-polygon_SDF(obstacle, (rbt + X[4])[:2]) + polygon_SDF(obstacle, (rbt + X[5])[:2])) / (2 * delta_t)
    return np.array([df_dx, df_dy, df_dt])

def sdf(rbt, tgt):
    rbt_d = map2display(map_origin, map_ratio, rbt.reshape((3, 1))).squeeze()
    # minimum vertex polygon function is called inside SDF_RT (raytracing)
    rt_visible = SDF_RT(rbt_d, psi, radius, 50, _global_map)
    visible_region = (display2map(map_origin, map_ratio, rt_visible.T)[0:2, :]).T
    # polygon_SDF has a -ve sign in front of the sdf for FOV (since we want to keep target inside the polygon)
    return polygon_SDF(visible_region, tgt[0:2])


def G(state):
    _, _, theta = state
    return np.array([[dt * np.cos(theta), 0],[dt * np.sin(theta), 0], [0, dt]])

def f(x, u):
    y = x + G(x) @ u
    return y

def clip_opt_u(u):
    u[0] = np.clip(u[0],v_lb,v_ub)
    u[1] = (u[1] + np.pi) % (2 * np.pi) - np.pi
    u[1] = np.clip(u[1], omega_lb, omega_ub)
    return u

def g_obs(rbt_state, u, obstacle):
    # use -polygonSDF as this fcn has extra -ve sign for FoV convention - for obstacles we need regular convention
    val1 = obstacle_fd_grad(rbt_state, obstacle) @ G(rbt_state)
    val2 = -polygon_SDF(obstacle, rbt_state[0:2])
    val3 = obstacle_fd_grad(rbt_state, obstacle)
    print("Robot SDF to obstacle with centre: ", np.mean(obstacle, axis=0), " = ", round(val2, 2))
    return (obstacle_fd_grad(rbt_state, obstacle) @ G(rbt_state) @ u) + (alpha_obs * -polygon_SDF(obstacle, rbt_state[0:2]))

def g_fov(rbt_state, tgt_state, u, ydot):
    val1 = new_fd_grad(rbt_state, tgt_state) @ G(rbt_state)
    val2 = alpha_fov * sdf(rbt_state, tgt_state)
    # note: use @ G @ u instead of @f since f uses x+G(x)u
    return (new_fd_grad(rbt_state, tgt_state) @ G(rbt_state) @ u) + (alpha_fov * sdf(rbt_state, tgt_state))

def is_obs_ahead(obstacle, rbt):
    eps = 10 # set eps higher to add obstacles more directly facing the agent; lower includes more obstacles
    # if any vertex is much closer to rbt's fov than rbt itself, the obstacle must be in the direction of rbt fov
    for vertex in obstacle:
        rbt_sdf = -sdf(rbt, vertex)
        if np.abs(rbt_sdf - np.linalg.norm(vertex[:2] - rbt[:2])) > eps or rbt_sdf < eps: # 2nd condition for case when
            # robot is right in front of obstacle and 1st condition evaluates to false but obstacle must be considered;
            # note the 2nd condition also catches obstacles that are behind the agent within eps distance
            return True
    return False

def is_valid_obstacle(obstacle, rbt):
    if obs_incl_cond == "distance":
        if np.linalg.norm(obstacle[0, :2] - rbt[0:2]) < radius/2:
            return True
    elif obs_incl_cond == "direction":
        if is_obs_ahead(obstacle, rbt):
            return True
    elif obs_incl_cond == "all":
        if is_obs_ahead(obstacle, rbt) and np.linalg.norm(obstacle[0, :2] - rbt[0:2]) < radius:
            return True
    return False

def solve_qp(rbt, tgt, tgt_dot, ref, R, list_obstacles, R_reg=None, u_prev=None):
    # note: can't use parameterized DPP since problem not affine in the parameters R, ref, rbt, tgt
    u = cp.Variable(2)
    M = 1
    # R = cp.Parameter((2, 2), PSD=True)
    if use_penalty_smoothing and sdf(rbt,tgt) >= 0:
        objective = cp.Minimize(cp.quad_form((u - ref), R) + cp.quad_form((u - np.mean(u_prev,axis=1)), R_reg))
        if use_fov_slack:
            objective = cp.Minimize(cp.quad_form((u - ref), R) + cp.quad_form((u - np.mean(u_prev, axis=1)), R_reg) +
                                    M*slack_delta**2)
    else:
        objective = cp.Minimize(cp.quad_form((u - ref), R))
        if use_fov_slack:
            objective = cp.Minimize(cp.quad_form((u - ref), R) + M*slack_delta**2)

    constraints = []
    # obstacle constraints
    # for obstacle in list_obstacles:
    #     if is_valid_obstacle(obstacle, rbt):
    #         constraints.append(g_obs(rbt,u,obstacle[:,:2]) >= 0)
    print("# of obstacle CBFs at current position: ", len(constraints))
    # visibility constraint
    if use_fov_slack:
        constraints.append(g_fov(rbt, tgt, u, tgt_dot) >= slack_delta)
    else:
        constraints.append(g_fov(rbt, tgt, u, tgt_dot) >= 0)
    # negative velocity constraint
    # constraints.append(u[0] >= 0)
    prob = cp.Problem(objective, constraints)
    prob.solve()
    if u.value is None:
        print('Infeasible solution...solving again without FoV constraint')
        u = cp.Variable(2)
        if use_penalty_smoothing and sdf(rbt, tgt) >= 0:
            objective = cp.Minimize(cp.quad_form((u - ref), R) + cp.quad_form((u - np.mean(u_prev, axis=1)), R_reg))
        else:
            objective = cp.Minimize(cp.quad_form((u - ref), R))
        # if infeasible due to target being in obstacle, solve the problem again with just obstacle constraints
        constraints = []
        for obstacle in list_obstacles:
            if is_valid_obstacle(obstacle, rbt):
                constraints.append(g_obs(rbt, u, obstacle[:, :2]) >= 0)
        prob = cp.Problem(objective, constraints)
        prob.solve()
    print('QP solve status: ', prob.status)
    u_clipped = clip_opt_u(copy(u.value))
    print(prob.value, "Original optimized control: ", u.value, "With control bounds: ", u_clipped)
    return u_clipped


if __name__ == "__main__":
    R = np.identity(2)
    R[0,0] = 0.1
    R[1,1] = 0.01
    ref = np.array([10,0])
    rbt = np.array([142.78570182, 0., 1.9])
    tgt = tgt_future_traj[:,0]
    obstacles = [np.array([[-20,-20],[-20,20],[20,20],[20,-20],[-20,-20]])]
    u = solve_qp(rbt, tgt, ref, R, obstacles)
