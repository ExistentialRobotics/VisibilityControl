import pickle
import socket
import warnings
from functools import lru_cache

import matplotlib.pyplot as plt
import numpy as np
# from polylabel import polylabel
from scipy.special import erf

from world.env.config import *

VisionRange = radius
ViewAngle = psi

global_map = np.zeros((1, 1))


def map2world(map_origin, ratio, x_m):
    if len(x_m) == 2:
        x_m = np.array([[1, 0], [0, 1], [0, 0]]) @ x_m
    map_origin = map_origin.reshape((3, 1))
    x_m = x_m.reshape((3, -1))
    m2w = np.array([[0, 1 / ratio, 0],
                    [-1 / ratio, 0, 0],
                    [0, 0, 1]])
    return m2w @ (x_m - map_origin)


def world2map(map_origin, ratio, x_w):
    map_origin = map_origin.reshape((3, 1))
    x_w = x_w.reshape((3, -1))
    w2m = np.array([[0, -ratio, 0],
                    [ratio, 0, 0],
                    [0, 0, 1]])
    return w2m @ x_w + map_origin


def SE2_kinematics(x, action, tau: float):
    wt_2 = action[1] * tau / 2
    t_v_sinc_term = tau * action[0] * np.sinc(wt_2 / np.pi)
    ret_x = np.empty(3)
    ret_x[0] = x[0] + t_v_sinc_term * np.cos(x[2] + wt_2)
    ret_x[1] = x[1] + t_v_sinc_term * np.sin(x[2] + wt_2)
    ret_x[2] = normalize_angle(x[2] + 2 * wt_2)
    return ret_x


def normalize_angle(angle):
    """Normalize an angle to be within the range [-π, π]"""
    warp_angle = (angle + np.pi) % (2 * np.pi) - np.pi
    # Handle the case when angle = -π so that the result is -π, not π
    if warp_angle == -np.pi and angle > 0:
        return np.pi
    return warp_angle


def min_distance(alpha, radius):
    return radius * np.sin(alpha) / (2 + 2 * np.sin(0.5 * alpha))


# def min_distance_new(fov_polygon):
#     center_pos, d_bar = polylabel(fov_polygon, with_distance=True)
#     return d_bar


def vertices_filter(polygon, angle_threshold=0.05):
    diff = polygon[1:] - polygon[:-1]
    diff_norm = np.sqrt(np.einsum('ij,ji->i', diff, diff.T))
    unit_vector = np.divide(diff, diff_norm[:, None], out=np.zeros_like(diff), where=diff_norm[:, None] != 0)
    angle_distance = np.round(np.einsum('ij,ji->i', unit_vector[:-1, :], unit_vector[1:, :].T), 5)
    angle_abs = np.abs(np.arccos(angle_distance))
    minimum_polygon = polygon[[True] + list(angle_abs > angle_threshold) + [True], :]
    return minimum_polygon


# def vertices_filter_jnp(polygon, angle_threshold=0.05):
#     minimum = jnp.zeros(jnp.shape(polygon))
#     minimum[0] = polygon[0]
#     counter = 1
#     diff = polygon[1:] - polygon[:-1]
#     for i in range(len(diff) - 1):
#         unit_vector_1 = diff[i] / jnp.linalg.norm(diff[i])
#         unit_vector_2 = diff[i + 1] / jnp.linalg.norm(diff[i + 1])
#         dot_product = jnp.dot(unit_vector_1, unit_vector_2)
#         angle = jnp.arccos(jnp.round(dot_product,5))
#         if abs(angle) > angle_threshold:
#             minimum[counter] = polygon[i + 1]
#             counter += 1
#     minimum[counter] = polygon[0]
#     return minimum[0:counter + 1]

def sigmoid(k, x):
    return 1 / (1 + np.exp(-k * x))


def plot_sigmoid_and_grad():
    pert = 10e-2
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex='all')

    m = 0.1
    for i in range(5):
        arr_x = np.linspace(-5, 5, 101)
        arr_sigmoid = sigmoid(arr_x, k=m)
        arr_fd_x = (sigmoid(arr_x + pert, k=m) - sigmoid(arr_x - pert, k=m)) / (2 * pert)
        ax1.plot(arr_x, arr_sigmoid)
        ax2.plot(arr_x, arr_fd_x)
        m += 1
    ax1.set_ylabel('Sigmoid(x)')
    ax2.set_ylabel('Gradient of Sigmoid(x)')
    ax2.set_xlabel('x : Euclidean distance of target from agent (0 is FoV boundary)')
    fig.suptitle('Sigmoid(x) and gradient as a function of target distance from agent \n and sigmoid parameter k')
    plt.show()


def triangle_SDF(q, psi, r):
    psi = 0.5 * psi
    r = r * np.cos(psi)
    x, y = q[0], q[1]
    p_x = r / (1 + np.sin(psi))

    a_1, a_2, a_3 = np.array([-1, 1 / np.tan(psi)]), np.array([-1, -1 / np.tan(psi)]), np.array([1, 0])
    b_1, b_2, b_3 = 0, 0, -r
    q_1, q_2, q_3 = np.array([r, r * np.tan(psi)]), np.array([r, -r * np.tan(psi)]), np.array([0, 0])
    # q_1, q_2, q_3 = np.array([r, r * np.sin(psi)]), np.array([r, -r * np.sin(psi)]), np.array([0, 0])
    l_1_low, l_1_up, l_2_low, l_2_up = l_function(x, psi, r, p_x)
    if y >= l_1_up:
        # P_1
        SDF, Grad = np.linalg.norm(q - q_1), (q - q_1) / np.linalg.norm(q - q_1)
    elif l_1_low <= y < l_1_up:
        # D_1
        SDF, Grad = (a_1 @ q + b_1) / np.linalg.norm(a_1), a_1 / np.linalg.norm(a_1)
    elif x < 0 and l_2_up <= y < l_1_low:
        # P_3
        SDF, Grad = np.linalg.norm(q - q_3), (q - q_3) / np.linalg.norm(q - q_3)
    elif x > p_x and l_2_up <= y < l_1_low:
        # D_3
        SDF, Grad = (a_3 @ q + b_3) / np.linalg.norm(a_3), a_3 / np.linalg.norm(a_3)
    elif l_2_up > y > l_2_low:
        # D_2
        SDF, Grad = (a_2 @ q + b_2) / np.linalg.norm(a_2), a_2 / np.linalg.norm(a_2)
    else:
        # P_2
        SDF, Grad = np.linalg.norm(q - q_2), (q - q_2) / np.linalg.norm(q - q_2)
    return SDF, Grad


def SDF_geo(x, alpha, r):
    vertex1, vertex2, vertex3 = np.array([0, 0]), np.array(
        [r * np.cos(0.5 * alpha), r * np.sin(0.5 * alpha)]), np.array(
        [r * np.cos(0.5 * alpha), -r * np.sin(0.5 * alpha)])
    distance = min(dist(vertex1, vertex2, x), dist(vertex2, vertex3, x), dist(vertex1, vertex3, x))
    return isInTriangle(vertex1, vertex2, vertex3, x) * distance


def isInTriangle(p1, p2, p3, x):
    x1, y1, x2, y2, x3, y3, xp, yp = np.hstack([p1, p2, p3, x])
    c1 = (x2 - x1) * (yp - y1) - (y2 - y1) * (xp - x1)
    c2 = (x3 - x2) * (yp - y2) - (y3 - y2) * (xp - x2)
    c3 = (x1 - x3) * (yp - y3) - (y1 - y3) * (xp - x3)
    if (c1 < 0 and c2 < 0 and c3 < 0) or (c1 > 0 and c2 > 0 and c3 > 0):
        return -1
    else:
        return 1


def dist(p1, p2, p3):  # x3,y3 is the point
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    px = x2 - x1
    py = y2 - y1

    norm = px * px + py * py

    u = ((x3 - x1) * px + (y3 - y1) * py) / float(norm)

    if u > 1:
        u = 1
    elif u < 0:
        u = 0

    x = x1 + u * px
    y = y1 + u * py

    dx = x - x3
    dy = y - y3

    dist = (dx * dx + dy * dy) ** .5

    return dist


def handle_warning(message, category, filename, lineno, file=None, line=None):
    print('A warning occurred:')
    print(message)
    print('Do you wish to continue?')

    while True:
        response = input('y/n: ').lower()
        if response not in {'y', 'n'}:
            print('Not understood.')
        else:
            break

    if response == 'n':
        raise category(message)


warnings.showwarning = handle_warning


def polygon_sdf(polygon, point):
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
        wn += 1 if cond1 and cond2 and np.cross(e[i], v[i]) > 0 else 0
        wn -= 1 if ~cond1 and ~cond2 and np.cross(e[i], v[i]) < 0 else 0
    sign = 1 if wn == 0 else -1
    return np.sqrt(d) * sign


def DDA(x0, y0, x1, y1, world_map):
    @lru_cache(None)
    def DDA_core(x0, y0, x1, y1):
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
            if 0 < int(x) < len(world_map) and 0 < int(y) < len(world_map[0]):
                if world_map[int(x), int(y)] == 0:
                    break
            x = x + x_inc
            y = y + y_inc
        return int(x) + 1, int(y) + 1

    return DDA_core(x0, y0, x1, y1)


def visible_region_omni(robot_pose, radius, RT_res, world_map):
    x0, y0, theta = robot_pose
    y_mid = [y0 + radius * np.sin(theta - np.pi + i * 2 * np.pi / RT_res) for i in range(RT_res + 1)]
    x_mid = [x0 + radius * np.cos(theta - np.pi + i * 2 * np.pi / RT_res) for i in range(RT_res + 1)]
    pts = []
    for i in range(len(x_mid)):
        xx, yy = DDA(int(x0), int(y0), int(x_mid[i]), int(y_mid[i]), world_map)
        if not pts or (yy != pts[-1][1] or xx != pts[-1][0]):
            pts.append([xx, yy])
    return np.array(pts)


def SDF_RT(robot_pose, fov, radius, RT_res, world_map, inner_r=10):
    pts = raytracing(robot_pose, fov, radius, RT_res, world_map)
    x0, y0, theta = robot_pose
    x1_inner = x0 + inner_r * np.cos(theta - 0.5 * fov)
    y1_inner = y0 + inner_r * np.sin(theta - 0.5 * fov)
    x2_inner = x0 + inner_r * np.cos(theta + 0.5 * fov)
    y2_inner = y0 + inner_r * np.sin(theta + 0.5 * fov)
    pts = [[x1_inner, y1_inner]] + pts + [[x2_inner, y2_inner], [x1_inner, y1_inner]]
    return vertices_filter(np.array(pts))
    # return np.array(pts)


def raytracing(robot_pose, fov, radius, RT_res, world_map):
    x0, y0, theta = robot_pose
    x1 = x0 + radius * np.cos(theta - 0.5 * fov)
    y1 = y0 + radius * np.sin(theta - 0.5 * fov)
    x2 = x0 + radius * np.cos(theta + 0.5 * fov)
    y2 = y0 + radius * np.sin(theta + 0.5 * fov)
    # y_mid = [y0 + radius * np.sin(theta - 0.5 * fov + i*fov / RT_res) for i in range(RT_res+1)]
    # x_mid = [x0 + radius * np.cos(theta - 0.5 * fov + i*fov / RT_res) for i in range(RT_res+1)]
    y_mid = np.linspace(y1, y2, RT_res)
    x_mid = np.linspace(x1, x2, RT_res)
    pts = []
    for i in range(len(x_mid)):
        xx, yy = DDA(int(x0), int(y0), int(x_mid[i]), int(y_mid[i]), world_map)
        if not pts or (yy != pts[-1][1] or xx != pts[-1][0]): pts.append([xx, yy])
    return pts





def dqxdx(x, y):
    theta = x[2]
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta), np.cos(theta)]])
    Rp = np.array([[-np.sin(theta), -np.cos(theta)],
                   [np.cos(theta), -np.sin(theta)]])
    Sp = np.array([[1, 0, 0],
                   [0, 1, 0]])
    e3 = np.array([[0], [0], [1]])
    return Rp.T @ Sp @ (x.reshape(3, 1) - y.reshape(3, 1)) @ e3.T + R.T @ Sp


def l_function(x, psi, r, p_x):
    if x < 0:
        l_1_low, l_2_up = - x / np.tan(psi), x / np.tan(psi)
    elif 0 <= x < p_x:
        l_1_low, l_2_up = 0, 0
    elif p_x <= x < r:
        l_1_low = np.tan(np.pi / 4 + psi / 2) * x - r / np.cos(psi)
        l_2_up = - np.tan(np.pi / 4 + psi / 2) * x + r / np.cos(psi)
    else:
        l_1_low, l_2_up = r * np.tan(psi), -r * np.tan(psi)

    if x < r:
        l_1_up, l_2_low = - (x - r) / np.tan(psi) + r * np.tan(psi), (x - r) / np.tan(psi) - r * np.tan(psi)
    else:
        l_1_up, l_2_low = r * np.tan(psi), -r * np.tan(psi)
    return l_1_low, l_1_up, l_2_low, l_2_up


