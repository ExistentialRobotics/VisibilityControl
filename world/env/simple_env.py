import os
import pickle
import time
from functools import lru_cache
import cv2
import matplotlib.pyplot as plt
import numpy as np
from world.env.maps.map import Map
from utils.utils import SE2_kinematics, polygon_sdf, triangle_SDF, SDF_RT, world2map, map2world
from world.npc.invader import Invader
from pathlib import Path

project_root = Path(__file__).resolve().parents[2]
RAY_TRACING_DEBUG = False


class Environment:
    def __init__(self, tau, psi, radius, epsilon_s):
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
        self._env_size = None
        self._theta_real = None
        self._epsilon_s = epsilon_s
        self._mu = None
        self._v = None
        self._landmark_motion_bias = None
        self._rbt = None
        self._psi = psi
        self._radius = radius
        self._tau = tau
        # r is radius for circular FoV centred at agent position
        self.r = 30
        # self.out_2D = cv2.VideoWriter('../2D.avi', fourcc, 15.0, (976, 780))

        self._frame_index = 0

    def reset(self, offset=0):
        trajectory_file = os.path.join(project_root, 'resources/Lsj.npz')
        self.invader = Invader(trajectory_file)
        self.invader.start()

        x = np.array([140, -100, -1.57])

        self._rbt = x

        self.history_poses = [self._rbt.tolist()]
        self.target_traj = []
        self.fig = plt.figure(1, figsize=(8, 8))
        self.ax = self.fig.gca()
        self.timeT = 0
        self.sim_tick = offset
        ob, self._obx, self._oby = [], [], []
        self._global_map = Map().map
        self.map_origin = np.array([[len(self._global_map) // 2], [len(self._global_map[0]) // 2], [np.pi / 2]])
        self.tgt_spd = 5
        self.map_ratio = 2

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
        rbt_d = world2map(self.map_origin, self.map_ratio, rbt.reshape((3, 1))).squeeze()
        rt_visible = SDF_RT(rbt_d, self._psi, self._radius, 50, self._global_map)
        visible_region = (map2world(self.map_origin, self.map_ratio, rt_visible.T)[0:2, :]).T
        return visible_region

    def get_polygon_from_rays(self, arr_rays):
        return

    def new_fd_grad(self, rbt, tgt):
        delta = 1.0
        delta_t = 0.01
        X = np.array(
            ([[delta, 0, 0], [-delta, 0, 0], [0, delta, 0], [0, -delta, 0], [0, 0, delta_t], [0, 0, -delta_t]]))
        df_dx = (self.sdf(rbt + X[0], tgt) - self.sdf(rbt + X[1], tgt)) / (2 * delta)
        df_dy = (self.sdf(rbt + X[2], tgt) - self.sdf(rbt + X[3], tgt)) / (2 * delta)
        df_dt = (self.sdf(rbt + X[4], tgt) - self.sdf(rbt + X[5], tgt)) / (2 * delta_t)
        return np.array([df_dx, df_dy, df_dt])

    def sdf(self, rbt, tgt):
        rbt_d = world2map(self.map_origin, self.map_ratio, rbt.reshape((3, 1))).squeeze()
        # minimum vertex polygon function is called inside SDF_RT (raytracing)
        rt_visible = SDF_RT(rbt_d, self._psi, self._radius, 50, self._global_map)
        visible_region = (map2world(self.map_origin, self.map_ratio, rt_visible.T)[0:2, :]).T
        return polygon_sdf(visible_region, tgt[0:2])


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
        # delta_x_ccw = np.array(
        #     [[r * np.cos(2 * np.pi * i / N), r * np.sin(2 * np.pi * i / N), delta_theta] for i in range(1, N + 1)])
        # delta_x_cw = np.array(
        #     [[r * np.cos(2 * np.pi * i / N), r * np.sin(2 * np.pi * i / N), -delta_theta] for i in range(1, N + 1)])
        # delta_x = np.vstack([delta_x_shift, delta_x_ccw, delta_x_cw])
        #
        rbt_d = world2map(self.map_origin, self.map_ratio, rbt.reshape((3, 1))).squeeze()
        rt_visible = SDF_RT(rbt_d, self._psi, self._radius, 50, self._global_map)
        visible_region = (map2world(self.map_origin, self.map_ratio, rt_visible.T)[0:2, :]).T

        SDF_center = polygon_sdf(visible_region, tgt[0:2])
        # X = delta_x
        # Y = np.zeros((X.shape[0], 1))
        Z = delta_x_shift[:, 0:2]
        W = np.zeros((delta_x_shift.shape[0], 1))
        # for index, dx in enumerate(X):
        #     rbt_d = map2display(self.map_origin, self.map_ratio, (rbt + dx).reshape((3, 1))).squeeze()
        #     rt_dx = SDF_RT(rbt_d, self._psi, self._radius, 50, self._global_map)
        #     visible_dx = (display2map(self.map_origin, self.map_ratio, rt_dx.T)[0:2, :]).T
        #     Y[index, 0] = polygon_SDF(visible_dx, tgt[0:2]) - SDF_center
        # grad_est = np.linalg.inv(X.T @ X) @ X.T @ Y
        # grad_est = -grad_est

        grad_est = self.new_fd_grad(rbt, tgt)

        for index, dy in enumerate(Z):
            W[index, 0] = polygon_sdf(visible_region, tgt[0:2] + dy) - SDF_center
        grad_est_y = -np.linalg.inv(Z.T @ Z) @ Z.T @ W
        return grad_est.reshape(3, ) + 1e-10, np.hstack([grad_est_y.reshape(2, ) + 1e-10, [0]]), SDF_center

    def cv_render(self, rbt, tgt, obs):
        tgt_d = world2map(self.map_origin, self.map_ratio, tgt.reshape((3, 1))).squeeze()

        rbt_cvx_d = world2map(self.map_origin, self.map_ratio, rbt.reshape((3, 1))).squeeze()
        rt_visible = SDF_RT(rbt_cvx_d, self._psi, self._radius, 50, self._global_map)
        visible_region_cvx = (map2world(self.map_origin, self.map_ratio, rt_visible.T)[0:2, :]).T
        SDF_center_cvx = polygon_sdf(visible_region_cvx, tgt[0:2])

        if obs is not None:
            obs_d = world2map(self.map_origin, self.map_ratio, np.hstack((obs, np.array([0]))).reshape((3, 1))).squeeze()
        else:
            obs_d = None

        visible_map = cv2.cvtColor(self._global_map, cv2.COLOR_GRAY2BGR)
        rt_visible = np.flip(rt_visible.astype(np.int32))
        visible_map = cv2.polylines(visible_map, [rt_visible.reshape(-1, 1, 2)], True,
                                    (255 if SDF_center_cvx < 0 else 128, 255 if SDF_center_cvx < 0 else 128, 0), 2)
        visible_map[int(tgt_d[0]) - 2:int(tgt_d[0]) + 2, int(tgt_d[1]) - 2:int(tgt_d[1]) + 2] = np.array([0, 0, 255])
        visible_map[int(rbt_cvx_d[0]) - 2:int(rbt_cvx_d[0]) + 2, int(rbt_cvx_d[1]) - 2:int(rbt_cvx_d[1]) + 2] = np.array([255, 0, 0])

        if obs_d is not None:
            visible_map[int(obs_d[0]) - 2:int(obs_d[0]) + 2, int(obs_d[1]) - 2:int(obs_d[1]) + 2] = np.array([255, 0, 255])

        visible_map = visible_map[100:-100, :, :]
        cv2.imshow('debugging', visible_map)
        self.fps_timer = time.time()
        key = (cv2.waitKey(1) & 0xFF) == ord('q')
        return key

    def update(self, action):
        self._rbt = SE2_kinematics(self._rbt, action, self._tau)
        self._tgt = self.invader.get_pose()
        self._tgt_dot = self.invader.get_velocity().reshape((3, 1))

        return self._tgt, self._tgt_dot, self._rbt

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

    def SDF_RT_circular(self, robot_pose, radius, RT_res):
        pts = self.raytracing_circular(robot_pose, 2 * np.pi, radius, RT_res)
        return np.array(pts)

    def raytracing_circular(self, robot_pose, fov, radius, RT_res):
        x0, y0, theta = robot_pose
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

    def close(self):
        if False: self.out_2D.release()
