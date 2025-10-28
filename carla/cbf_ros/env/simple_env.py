import pickle
import time
from functools import lru_cache

# import shapely
import numpy as np
import cv2
from env.config import *
import matplotlib.pyplot as plt
from utils import SE2_kinematics, polygon_SDF, triangle_SDF, SDF_RT, load_local_traj, map2display, display2map, sigmoid, sig_k
from env.map.map import Map
from utils_planner import boundaries

# import visilibity as vis

# matplotlib.use('TkAgg')
# UDP_IP = "127.0.0.1"
# UDP_PORT = 5005
# fourcc = cv2.VideoWriter_fourcc(*'XVID')

RAY_TRACING_DEBUG = False


class Environment:
    def __init__(self, agent, horizon, tau, psi, radius, epsilon_s, obs, wls):
        self.agent = agent
        self.fps_timer = time.time()
        self.tgt_spd = None
        self.sim_tick = None
        self.sock = None
        self._global_map = None
        self._detectAgt = None
        self._detectTgt = None
        self.TargetVisibility = None
        self.AgentVisibility = None
        self._obsEnv = None
        self.timeT = None
        self.target_traj = None
        self._tgt_dot = None
        self.ax = None
        self.fig = None
        self.history_poses = None
        self._tgt = None
        self._omega = None
        self._mu_real = None
        self._horizon = horizon
        self._env_size = None
        self._theta_real = None
        self._epsilon_s = epsilon_s
        self._tau = tau
        self._obstacles = obs
        self._walls = wls

        self._mu = None
        self._v = None
        self._landmark_motion_bias = None
        self._rbt = None
        self._step_num = None

        self._psi = psi
        self._radius = radius
        # r is radius for circular FoV centred at agent position
        self.r = 30
        # self.out_2D = cv2.VideoWriter('../2D.avi', fourcc, 15.0, (976, 780))

        self._frame_index = 0

    def reset(self, offset=0):
        self._env_size = np.array([4, 4])
        self._horizon = self._horizon
        # mu = (np.random.random((1, 2)) - 0.5) * self._env_size
        # theta = np.random.random((1, 1)) - 0.5
        x = np.array([140, -100, -1.57])
        trajectory_file = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, "robot/planner/Lsj.npz"))
        self.tgt_traj = load_local_traj(trajectory_file)
        self.tgt_traj[:, 2] = np.where(self.tgt_traj[:, 2] < 0, self.tgt_traj[:, 2] + 2 * np.pi, self.tgt_traj[:, 2])

        # self._mu_real = mu
        # self._theta_real = theta
        self._rbt = x
        self._step_num = 0
        self._tgt = self.tgt_traj[0][:3]  # np.hstack((self._mu_real, self._theta_real))[0]

        self.history_poses = [self._rbt.tolist()]
        self.target_traj = []
        self.fig = plt.figure(1, figsize=(8, 8))
        self.ax = self.fig.gca()
        self.timeT = 0
        self.sim_tick = offset
        ob, self._obx, self._oby = [], [], []
        self._global_map = Map().map
        self.map_origin = np.array([[len(self._global_map) // 2], [len(self._global_map[0]) // 2], [np.pi / 2]])
        # self.sock = socket.socket(socket.AF_INET,  # Internet
        #                           socket.SOCK_DGRAM)  # UDP
        # self.sock.bind((UDP_IP, UDP_PORT))
        self.tgt_spd = 5
        self.map_ratio = 2
        # self.agent_traj_local = load_local_traj('../results/robot_traj_2D.npz')
        # self.sock = sock_init("127.0.0.1", 5005, 'r')

    def SDF_RT_circular(self, robot_pose, radius, RT_res, slam_map):
        self._global_map = slam_map
        pts = self.raytracing_circular(robot_pose, 2 * np.pi, radius, RT_res)
        return np.array(pts)

    def raytracing_circular(self, robot_pose, fov, radius, RT_res):
        x0, y0, theta = robot_pose
        out_1, out_2 = boundaries(robot_pose, fov, radius)
        y_mid = [y0 + radius * np.sin(theta - 0.5 * fov + i * fov / RT_res) for i in range(RT_res + 1)]
        x_mid = [x0 + radius * np.cos(theta - 0.5 * fov + i * fov / RT_res) for i in range(RT_res + 1)]
        pts = []
        for i in range(len(x_mid)):
            xx, yy = self.DDA(int(x0), int(y0), int(x_mid[i]), int(y_mid[i]))
            if not pts or (yy != pts[-1][1] or xx != pts[-1][0]):
                pts.append([xx, yy])
        return pts

    @lru_cache(None)
    def DDA(self, x0, y0, x1, y1):
        # find absolute differences
        dx = x1 - x0
        dy = y1 - y0

        # find maximum difference
        steps = int(max(abs(dx), abs(dy)))

        # calculate the increment in x and y
        x_inc = dx / steps
        y_inc = dy / steps

        # start with 1st point
        x = float(x0)
        y = float(y0)

        for i in range(steps):
            if 0 < int(x) < len(self._global_map) and 0 < int(y) < len(self._global_map[0]):
                if self._global_map[int(x), int(y)] == 0:
                    break
            x = x + x_inc
            y = y + y_inc
        return int(x) + 1, int(y) + 1

    def save_print(self, polygon):
        end_pos_x = []
        end_pos_y = []
        # print('Points of Polygon: ')
        for i in range(polygon.n()):
            x = polygon[i].x()
            y = polygon[i].y()
            end_pos_x.append(x)
            end_pos_y.append(y)
        return end_pos_x, end_pos_y

    def getSDF(self, pose1, pose2):
        x1, x2 = pose1[:2][:, None], pose2[:2][:, None]
        R1 = np.array([[np.cos(pose1[2]), np.sin(-pose1[2])], [np.sin(pose1[2]), np.cos(pose1[2])]])
        R2 = np.array([[np.cos(pose2[2]), np.sin(-pose2[2])], [np.sin(pose2[2]), np.cos(pose2[2])]])
        x1p2 = np.matmul(R2.T, x1 - x2).T[0]
        x2p1 = np.matmul(R1.T, x2 - x1).T[0]
        SDF1, GRAD1 = triangle_SDF(x2p1, self._psi, self._radius)
        SDF2, GRAD2 = triangle_SDF(x1p2, self._psi, self._radius)
        return SDF1, SDF2, GRAD1, GRAD2

    def get_fov(self, x, r, p):
        v1 = np.array([x[0], x[1]])
        v2 = r * np.array(
            [np.cos(x[2] - 0.5 * p), np.sin(x[2] - 0.5 * p)]) + v1
        v3 = r * np.array(
            [np.cos(x[2] + 0.5 * p), np.sin(x[2] + 0.5 * p)]) + v1
        FOV = [v1, v2, v3]
        return FOV

    def check_if_contained(self, poly1, poly2, p1, p2):
        return poly1.contains(p2), poly2.contains(p1)

    def get_agent_pose(self, sock):
        pose, addr = sock.recvfrom(4096)  # buffer size is 1024 bytes
        if not pose: return [0, 0, 0]
        pose = pickle.loads(pose)
        return pose

    def get_visible_region(self, rbt, tgt):
        rbt_d = map2display(self.map_origin, self.map_ratio, rbt.reshape((3, 1))).squeeze()
        rt_visible = SDF_RT(rbt_d, self._psi, self._radius, 50, self._global_map)
        visible_region = (display2map(self.map_origin, self.map_ratio, rt_visible.T)[0:2, :]).T
        return visible_region

    def get_polygon_from_rays(self, arr_rays):
        return

    def new_fd_grad(self, rbt, tgt):
        delta = 1.0
        delta_t = 0.1
        X = np.array(
            ([[delta, 0, 0], [-delta, 0, 0], [0, delta, 0], [0, -delta, 0], [0, 0, delta_t], [0, 0, -delta_t]]))
        df_dx = (self.sdf(rbt + X[0], tgt) - self.sdf(rbt + X[1], tgt)) / (2 * delta)
        df_dy = (self.sdf(rbt + X[2], tgt) - self.sdf(rbt + X[3], tgt)) / (2 * delta)
        df_dt = (self.sdf(rbt + X[4], tgt) - self.sdf(rbt + X[5], tgt)) / (2 * delta_t)
        return -np.array([df_dx, df_dy, df_dt])

    def sdf(self, rbt, tgt):
        rbt_d = map2display(self.map_origin, self.map_ratio, rbt.reshape((3, 1))).squeeze()
        # minimum vertex polygon function is called inside SDF_RT (raytracing)
        rt_visible = SDF_RT(rbt_d, self._psi, self._radius, 50, self._global_map)
        visible_region = (display2map(self.map_origin, self.map_ratio, rt_visible.T)[0:2, :]).T
        return polygon_SDF(visible_region, tgt[0:2])

    def new_fd_grad_sig(self, rbt, tgt):
        delta = 10e-2
        delta_t = delta / 10
        X = np.array(
            ([[delta, 0, 0], [-delta, 0, 0], [0, delta, 0], [0, -delta, 0], [0, 0, delta_t], [0, 0, -delta_t]]))
        df_dx = (sigmoid(sig_k, self.sdf(rbt + X[0], tgt)) - sigmoid(sig_k, self.sdf(rbt + X[1], tgt))) / (2 * delta)
        df_dy = (sigmoid(sig_k, self.sdf(rbt + X[2], tgt)) - sigmoid(sig_k, self.sdf(rbt + X[3], tgt))) / (2 * delta)
        df_dt = (sigmoid(sig_k, self.sdf(rbt + X[4], tgt)) - sigmoid(sig_k, self.sdf(rbt + X[5], tgt))) / (2 * delta_t)
        return np.array([df_dx, df_dy, df_dt])

    def new_fd_hess(self, rbt, tgt):
        t0 = time.time()
        delta = 1
        delta_t = 0.1
        X = np.array(
            ([[x, y, t] for x in (delta, -delta, 0) for y in (delta, -delta, 0) for t in (delta_t, -delta_t, 0)]))
        SDF = self.sdf(rbt, tgt)
        d2f_dx2 = (self.sdf(rbt + X[8], tgt) + self.sdf(rbt + X[17], tgt) - 2 * SDF) / (delta ** 2)
        d2f_dy2 = (self.sdf(rbt + X[20], tgt) + self.sdf(rbt + X[23], tgt) - 2 * SDF) / (delta ** 2)
        d2f_dt2 = (self.sdf(rbt + X[24], tgt) + self.sdf(rbt + X[25], tgt) - 2 * SDF) / (delta_t ** 2)

        d2f_dxdy = (self.sdf(rbt + X[2], tgt) + self.sdf(rbt + X[14], tgt) -
                    self.sdf(rbt + X[5], tgt) - self.sdf(rbt + X[11], tgt)) / (4 * delta ** 2)
        d2f_dxdt = (self.sdf(rbt + X[6], tgt) + self.sdf(rbt + X[16], tgt) -
                    self.sdf(rbt + X[7], tgt) - self.sdf(rbt + X[15], tgt)) / (4 * delta * delta_t)
        d2f_dydt = (self.sdf(rbt + X[18], tgt) + self.sdf(rbt + X[22], tgt) -
                    self.sdf(rbt + X[19], tgt) - self.sdf(rbt + X[21], tgt)) / (4 * delta * delta_t)

        hessian = np.array([[d2f_dx2, d2f_dxdy, d2f_dxdt],
                            [d2f_dxdy, d2f_dy2, d2f_dydt],
                            [d2f_dxdt, d2f_dydt, d2f_dt2]])
        # print(time.time() - t0)
        return hessian

    def sdf_circular(self, rbt, tgt):
        return np.linalg.norm(rbt[0:2] - tgt[0:2]) - self.r

    def grad_sdf_circular(self, rbt, tgt):
        grad_x = (rbt[0] - tgt[0]) / np.linalg.norm(rbt[0:2] - tgt[0:2])
        grad_y = (rbt[1] - tgt[1]) / np.linalg.norm(rbt[0:2] - tgt[0:2])
        grad_th = 0
        return np.array([grad_x, grad_y, 0])

    def polygon_sdf_grad_lse(self, rbt, tgt, debug=False):

        r = 2
        N = 4
        delta_theta = 0.1
        delta_x_shift = np.array(
            [[r * np.cos(2 * np.pi * i / N), r * np.sin(2 * np.pi * i / N), 0] for i in range(1, N + 1)])
        rbt_d = map2display(self.map_origin, self.map_ratio, rbt.reshape((3, 1))).squeeze()
        rt_visible = SDF_RT(rbt_d, self._psi, self._radius, 50, self._global_map)
        visible_region = (display2map(self.map_origin, self.map_ratio, rt_visible.T)[0:2, :]).T

        SDF_center = polygon_SDF(visible_region, tgt[0:2])
        Z = delta_x_shift[:, 0:2]
        W = np.zeros((delta_x_shift.shape[0], 1))
        grad_est = self.new_fd_grad(rbt, tgt)

        for index, dy in enumerate(Z):
            W[index, 0] = polygon_SDF(visible_region, tgt[0:2] + dy) - SDF_center
        grad_est_y = -np.linalg.inv(Z.T @ Z) @ Z.T @ W
        return grad_est.reshape(3, ) + 1e-10, np.hstack([grad_est_y.reshape(2, ) + 1e-10, [0]]), SDF_center

    def cv_render(self, rbt, tgt):
        tgt_d = map2display(self.map_origin, self.map_ratio, tgt.reshape((3, 1))).squeeze()
        rbt_d = map2display(self.map_origin, self.map_ratio, rbt.reshape((3, 1))).squeeze()
        rt_visible = SDF_RT(rbt_d, self._psi, self._radius, 50, self._global_map)
        visible_region = (display2map(self.map_origin, self.map_ratio, rt_visible.T)[0:2, :]).T
        SDF_center = polygon_SDF(visible_region, tgt[0:2])
        if RAY_TRACING_DEBUG:
            rt_map = cv2.cvtColor(self._global_map, cv2.COLOR_GRAY2BGR)
            for i in range(len(rt_visible) - 2):
                rt_map = cv2.line(rt_map, tuple(rt_visible[0]), tuple(rt_visible[i + 1]), (50, 255, 0), 1)
                rt_map = rt_map[100:-100, :, :]
                cv2.imshow('ray-tracing', rt_map)
        visible_map = cv2.cvtColor(self._global_map, cv2.COLOR_GRAY2BGR)
        rt_visible = np.flip(rt_visible.astype(np.int32))
        visible_map = cv2.polylines(visible_map, [rt_visible.reshape(-1, 1, 2)], True,
                                    (50, 255 if SDF_center < 0 else 128, 0), 2)
        visible_map[int(tgt_d[0]) - 2:int(tgt_d[0]) + 2,
        int(tgt_d[1]) - 2:int(tgt_d[1]) + 2] = np.array([0, 0, 255])
        visible_map[int(rbt_d[0]) - 2:int(rbt_d[0]) + 2,
        int(rbt_d[1]) - 2:int(rbt_d[1]) + 2] = np.array([255, 0, 0])
        visible_map = visible_map[100:-100, :, :]
        cv2.imshow('debugging', visible_map)
        self.fps_timer = time.time()
        key = cv2.waitKey(1) & 0xFF
        return key

    def update(self, action):
        self.sim_tick += self.tgt_spd
        v, w = action
        # self._rbt = self.agent.f(0, self._rbt, v, w)
        self._rbt = SE2_kinematics(self._rbt, action, self._tau)
        # self._target_pose, self._target_traj_dot = Lissajous(A, B, a, b, delta, self.timeT)
        # time_t = int(self.timeT * 60) % len(self.traj_local)
        time_t_1 = int(time.time() * 15 + 200)
        self._tgt = self.tgt_traj[self.sim_tick % len(self.tgt_traj), :3]
        self._tgt_dot = self.tgt_traj[self.sim_tick % len(self.tgt_traj), 3:, None]
        tgt_display = self.tgt_traj[(self.sim_tick + 50) % len(self.tgt_traj), :3]
        # self._target_pose, self._target_traj_dot = self.target_traj_local[self.timeT2, :3], np.zeros((3, 1))
        # self._agent_pose = self.agent_traj_local[self.timeT2]
        return self._tgt.squeeze(), self._tgt_dot, self._rbt, tgt_display

    def target_future(self, horizon):
        y_future = self.tgt_traj[[(self.tgt_spd * tick + self.sim_tick) % len(self.tgt_traj) for tick in
                                  range(horizon)], :3]
        ydot_future = self.tgt_traj[[(self.tgt_spd * tick + self.sim_tick) % len(self.tgt_traj) for tick in
                                     range(horizon)], 3:]
        return y_future, ydot_future

    def step(self):
        self._mu_real = self._tgt[:2]
        self._theta_real = self._tgt[2:]
        self._v = np.random.randn(1) + 2
        self._omega = np.random.randn(1) + 0.5
        self._target_u = np.vstack((self._v, self._omega))
        # self._rbt = np.array([0, 0, 0])
        # self._tgt = np.array([50, 0, 0])

        Grad_A2T, Grad_A2Ty, SDF_A2T = self.polygon_sdf_grad_lse(self._rbt, self._tgt, True)
        # Grad_T2A, Grad_T2Ay, SDF_T2A = self.polygon_sdf_grad_lse(self._target_pose, self._agent_pose, False)
        H = self.new_fd_hess(self._rbt, self._tgt)
        self._step_num += 1
        done = self._step_num >= self._horizon

        self.target_traj.append(self._mu_real)
        self.history_poses.append(self._rbt.tolist())
        self.timeT += self._tau
        return done, SDF_A2T, Grad_A2T, 0, Grad_A2Ty

    def _plot(self, legend, SDF, SDF_inv, title='trajectory'):
        # x = self._agent_pose
        # tu, th = self._mu_real, self._theta_real[0]

        plt.tick_params(labelsize=15)
        history_poses = np.array(self.history_poses)  # with the shape of (1, 2)
        target_traj = np.array(self.target_traj)
        self.ax.plot(history_poses[:, 0], history_poses[:, 1], c='black', linewidth=3, label='agent trajectory')
        self.ax.plot(target_traj[:, 0], target_traj[:, 1], c='blue', linewidth=3, label='target trajectory')

        self.ax.fill(self.AgentVisibility[:, 0], self.AgentVisibility[:, 1],
                     alpha=0.7 if self._detectTgt else 0.3,
                     color='r')
        self.ax.fill(self.TargetVisibility[:, 0], self.TargetVisibility[:, 1],
                     alpha=0.7 if self._detectAgt else 0.3,
                     color='b')

        for k in range(len(self._obstacles)):
            self.ax.fill(self._obx[k], self._oby[k], c=[0, 0, 0], alpha=0.8)

        # plot agent trajectory start & end
        # self.ax.scatter(history_poses[0, 0], history_poses[0, 1], marker='>', s=70, c='red', label="start")
        # self.ax.scatter(history_poses[-1, 0], history_poses[-1, 1], marker='s', s=70, c='red', label="end")

        self.ax.scatter(history_poses[-1, 0] + np.cos(history_poses[-1, 2]) * 0.5,
                        history_poses[-1, 1] + np.sin(history_poses[-1, 2]) * 0.5, marker='o', c='black')

        # axes
        self.ax.set_xlabel("x", fontdict={'size': 20})
        self.ax.set_ylabel("y", fontdict={'size': 20})
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)

        # title
        # self.ax.set_title(title, fontdict={'size': 16})

        self.ax.set_facecolor('whitesmoke')
        plt.grid(alpha=0.4)
        # legend
        if legend:
            self.ax.legend()
            plt.legend(prop={'size': 8})
        self.ax.set_title(r'$SDF_{A\rightarrow T}=%f, SDF_{T\rightarrow A}=%f$' % (SDF, SDF_inv))

    def render(self, SDF, SDF_inv, mode='human'):
        self.ax.cla()

        # plot
        self._plot(True, SDF, SDF_inv)

        # display
        plt.savefig('../results/' + str(self._frame_index) + '.png')
        self._frame_index += 1
        plt.draw()
        plt.pause(0.01)

    def close(self):
        if False: self.out_2D.release()


if __name__ == '__main__':
    # obstacles = [[(2, -9), (2, -7), (4, -7), (4, -9)],
    #              [(-5, 7), (-5, 9), (-3, 9), (-3, 7)],
    #              [(8, 8), (8, 10), (10, 10), (10, 8)],
    #              [(-10, -9), (-10, -7), (-8, -7), (-8, -9)],
    #              [(-5, 16), (-5, 19), (-2, 19), (-2, 16)],
    #              [(-5, -19), (-5, -16), (-2, -16), (-2, -19)],
    #              [(-1, -3), (-2, 0), (-1, 3), (1, 2)]]
    obstacles = []
    walls = [(-20, -20), (20, -20), (20, 20), (-20, 20)]
    agt, tgt = np.array([0, 0, 0 * np.pi]), np.array([5, 0, 1.25 * np.pi])

    E = Environment(horizon=5000, tau=0.1, psi=1.0471975511965976, radius=10, epsilon_s=0.99, obs=obstacles, wls=walls)
    E.reset()
    # grad1 = E.polygon_SDF_Grad_LSE(agt, tgt)
    # SDF1, SDF2, Grad1, Grad2 = E.getSDF(agt, tgt)
    R = np.array([[np.cos(agt[2]), -np.sin(agt[2])], [np.sin(agt[2]), np.cos(agt[2])]])
    # grad_poly = R.T @ grad1

    pos = [(0.5 * i - 0.5, 0.5 * j + 0.5) for i in range(-30, 31, 1) for j in range(-30, 31, 1)]
    data = []

    result = []
    # plot = plt.figure(1, (8, 8))
    # plt.figure(1, (8, 8))
    # ax = plot.gca()
    # vertexAgent1 = np.array([0, 0])
    # vertexAgent2 = E._radius * np.array([np.cos(0.5 * E._psi), np.sin(-0.5 * E._psi)])
    # vertexAgent3 = E._radius * np.array([np.cos(0.5 * E._psi), np.sin(0.5 * E._psi)])
    # FOVAgent = Polygon([vertexAgent1, vertexAgent2, vertexAgent3], alpha=0.7)
    # ax.add_patch(FOVAgent)
    # ax.set_xlim(-20, 20)
    # ax.set_ylim(-20, 20)
    for p in pos:
        grad, grad_y, _ = E.polygon_sdf_grad_lse(agt, p)
        g = (R.T @ grad[0:2]).reshape(2, )
        # plt.arrow(p[0], p[1], g[0], g[1], head_width=0.2)
        data.append(grad[2])
        # plt.scatter(p[0],p[1],c=[])
    # plt.show()
    data = np.array(data).reshape((61, 61))
    plt.imshow(data, cmap='hot', interpolation='nearest')
    plt.show()
    pass
