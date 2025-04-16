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

def display2map(map_origin, ratio, x_d):
    if len(x_d) == 2:
        x_d = np.array([[1, 0], [0, 1], [0, 0]]) @ x_d
    d2m = np.array([[0, 1 / ratio, 0],
                    [-1 / ratio, 0, 0],
                    [0, 0, 1]])
    return d2m @ (x_d - map_origin)


def map2display(map_origin, ratio, x_m):
    m2d = np.array([[0, -ratio, 0],
                    [ratio, 0, 0],
                    [0, 0, 1]])
    return m2d @ x_m + map_origin


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

def sigmoid(k,x):
    return 1/(1+np.exp(-k*x))

def plot_sigmoid_and_grad():
    pert = 10e-2
    fig, (ax1, ax2) = plt.subplots(2,1, sharex='all')

    m=0.1
    for i in range(5):
        arr_x = np.linspace(-5, 5, 101)
        arr_sigmoid = sigmoid(arr_x, k=m)
        arr_fd_x = (sigmoid(arr_x + pert, k=m) - sigmoid(arr_x - pert, k=m)) / (2 * pert)
        ax1.plot(arr_x, arr_sigmoid)
        ax2.plot(arr_x, arr_fd_x)
        m+=1
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


def polygon_SDF(polygon, point):
    N = len(polygon) - 1
    e = polygon[1:] - polygon[:-1]
    v = point - polygon[:-1]
    pq = v - e * np.clip((v[:, 0] * e[:, 0] + v[:, 1] * e[:, 1]) /
                         (e[:, 0] * e[:, 0] + e[:, 1] * e[:, 1]), 0, 1).reshape(N, -1)
    d = np.min(pq[:, 0] * pq[:, 0] + pq[:, 1] * pq[:, 1])
    wn = 0
    for i in range(N):
        val3 = np.cross(e[i], v[i])
        i2 = int(np.mod(i + 1, N))
        cond1 = 0 <= v[i, 1]
        cond2 = 0 > v[i2, 1]
        wn += 1 if cond1 and cond2 and val3 > 0 else 0
        wn -= 1 if ~cond1 and ~cond2 and val3 < 0 else 0
    sign = 1 if wn == 0 else -1
    return np.sqrt(d) * sign


@lru_cache(None)
def DDA(x0, y0, x1, y1):
    # find absolute differences
    dx = x1 - x0
    dy = y1 - y0

    # find maximum difference
    steps = int(max(abs(dx), abs(dy)))

    # calculate the increment in x and y
    xinc = dx / steps
    yinc = dy / steps

    # start with 1st point
    x = float(x0)
    y = float(y0)

    for i in range(steps):
        if 0 < int(x) < len(global_map) and 0 < int(y) < len(global_map[0]):
            if global_map[int(x), int(y)] == 0:
                break
        x = x + xinc
        y = y + yinc
    return int(x) + 1, int(y) + 1


def SDF_RT(robot_pose, fov, radius, RT_res, grid, inner_r=10):
    global global_map
    global_map = grid
    pts = raytracing(robot_pose, fov, radius, RT_res)
    x0, y0, theta = robot_pose
    x1_inner = x0 + inner_r * np.cos(theta - 0.5 * fov)
    y1_inner = y0 + inner_r * np.sin(theta - 0.5 * fov)
    x2_inner = x0 + inner_r * np.cos(theta + 0.5 * fov)
    y2_inner = y0 + inner_r * np.sin(theta + 0.5 * fov)
    pts = [[x1_inner, y1_inner]] + pts + [[x2_inner, y2_inner], [x1_inner, y1_inner]]
    return vertices_filter(np.array(pts))
    # return np.array(pts)


def raytracing(robot_pose, fov, radius, RT_res):
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
        xx, yy = DDA(int(x0), int(y0), int(x_mid[i]), int(y_mid[i]))
        if not pts or (yy != pts[-1][1] or xx != pts[-1][0]): pts.append([xx, yy])
    return pts


def state_to_T(state):
    return np.array([[np.cos(state[2]), -np.sin(state[2]), state[0]], [np.sin(state[2]), np.cos(state[2]), state[1]],
                     [0, 0, 1]])


def Gaussian_CDF(x, kap):
    Psi = (1 + erf(x / (np.sqrt(2) * kap) - 2)) / 2
    Psi_der = 1 / (np.sqrt(2 * np.pi) * kap) * np.exp(- (x / (np.sqrt(2) * kap) - 2) ** 2)
    return Psi, Psi_der


def Ddqy(x, y, n_y, r, kap, std):
    y = y.squeeze()
    V_jj_inv = np.zeros((2 * n_y, 2 * n_y))
    T_pose = state_to_T(x)
    for j in range(n_y):
        q = T_pose[:2, :2].transpose() @ (y[j * 2: j * 2 + 2] - T_pose[:2, 2])
        SDF, Grad = triangle_SDF(q, np.pi / 3, r)
        Phi, Phi_der = Gaussian_CDF(SDF, kap)
        V_jj_inv[2 * j, 2 * j] = 1 / (std ** 2) * (1 - Phi)
        V_jj_inv[2 * j + 1, 2 * j + 1] = 1 / (std ** 2) * (1 - Phi)
    return np.ones((1, 2)) * V_jj_inv[0, 0]


def Lissajous(A, B, a, b, delta, t):
    x = A * np.sin(a * t + delta)
    y = B * np.sin(b * t)
    theta = np.arctan2(B * b * np.cos(b * t), A * a * np.cos(a * t + delta))
    dy = B * b * np.cos(b * t)
    dx = A * a * np.cos(a * t + delta)
    dtheta = 1 / (1 + (dy / dx) ** 2) * (
            (B * b * (a * np.sin(a * t + delta) * np.cos(b * t) - b * np.sin(b * t) * np.cos(a * t + delta))) / (
            A * a * (np.cos(a * t + delta)) ** 2))
    return np.array([x, y, theta]), np.array([dx, dy, dtheta]).reshape((3, 1))


def Lissajous_local(path, index):
    traj = np.load(path)['arr_0']
    return traj[index, :3], traj[index, 3:].reshape(3, 1)


def dqydy(x, y):
    theta = y[2]
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    Rp = np.array([[-np.sin(theta), -np.cos(theta)], [np.cos(theta), -np.sin(theta)]])
    Sp = np.array([[1, 0, 0], [0, 1, 0]])
    et = np.array([[0], [0], [1]])
    return Rp.T @ Sp @ (y.reshape(3, 1) - x.reshape(3, 1)) @ et.T + R.T @ Sp


def dqydx(x, y):
    theta = y[2]
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    Sp = np.array([[1, 0, 0], [0, 1, 0]])
    return -R.T @ Sp


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


def dqxdy(x, y):
    theta = x[2]
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    Sp = np.array([[1, 0, 0], [0, 1, 0]])
    return -R.T @ Sp


def doty(y_t):
    A, B, a, b, delta = 15, 15, 0.3, 0.7, 0.5 * np.pi
    dy = B * b * np.cos(b * y_t)
    dx = A * a * np.cos(a * y_t + delta)
    dtheta = 1 / (1 + (dy / dx) ** 2) * ((B * b * (
            a * np.sin(a * y_t + delta) * np.cos(b * y_t) - b * np.sin(b * y_t) * np.cos(a * y_t + delta))) / (
                                                 A * a * (np.cos(a * y_t + delta)) ** 2))
    return np.array([[dx], [dy], [dtheta]])


def doty2(y_t):
    tau = 0.1
    A, B, a, b, delta = 15, 15, 0.3, 0.7, 0.5 * np.pi
    target_x = A * np.sin(a * y_t + delta)
    target_y = B * np.sin(b * y_t)
    target_theta = np.arctan2(B * b * np.cos(b * y_t), A * a * np.cos(a * y_t + delta))
    y_t += tau
    target_x_1 = A * np.sin(a * y_t + delta)
    target_y_1 = B * np.sin(b * y_t)
    target_theta_1 = np.arctan2(B * b * np.cos(b * y_t), A * a * np.cos(a * y_t + delta))
    return np.array([[target_x_1 - target_x], [target_y_1 - target_y], [target_theta_1 - target_theta]]) / tau


def F_function(x):
    return np.array([[0], [0], [0]])


def G_function(x):
    theta = x[2]
    return np.array([[np.cos(theta), 0], [np.sin(theta), 0], [0, 1]])


def refInput(u):
    return u


def targetInput():
    return np.array([[0], [0], [0]])


def hH(d):
    epsilon_H = 10
    return d - epsilon_H


def hS(d):
    return -d - epsilon_s * min_distance(ViewAngle, VisionRange)


def L_Gh(x, y, grad):
    return grad.reshape(1, 2) @ dqxdx(x, y) @ G_function(x)


def L_Gh_new(x, rdrx):
    return (rdrx @ G_function(x)).reshape(1, 2)


def c_function_h(x, y, u, v, d, grad):
    alpha = 3
    mat = grad @ (dqydx(x, y) @ (F_function(x) + G_function(x) @ refInput(u)) + dqydy(x, y) @ (
            F_function(y) + G_function(y) @ v))
    return mat + alpha * hH(d) * np.ones(np.shape(mat))


def c_function_s(x, y, ydot, u, d, grad):
    alpha = 3
    mat = -grad @ (dqxdx(x, y) @ (F_function(x) + G_function(x) @ refInput(u)) + dqxdy(x, y) @ ydot)
    return mat + alpha * hS(d) * np.ones(np.shape(mat))


def c_function_s_new(x, ydot, u, d, rdrx, rdry, grad, y):
    alpha = 3
    mat = -(rdrx @ F_function(x) + rdrx @ G_function(x) @ refInput(u) + rdry @ ydot)
    # diff = rdrx - grad @ dqxdx(x, y)
    # print(diff)
    return mat + alpha * hS(d) * np.ones(np.shape(mat))


def c_direct(x, y, ydot, alpha, d, eps_s, grad):
    ydot = ydot.reshape(3)
    return -grad @ np.array([[-ydot[0] * np.cos(x[2]) - ydot[1] * np.sin(x[2])],
                             [ydot[0] * np.cos(x[2]) - ydot[1] * np.sin(x[2])]]) + alpha * (-d - eps_s)


def ubar_direct(c, x, y, grad):
    L_direct = (-grad @ np.array([[1, -(x[0] - y[0]) * np.sin(x[2]) + (x[1] - y[1]) * np.cos(x[2])],
                                  [0, -(x[0] - y[0]) * np.cos(x[2]) - (x[1] - y[1]) * np.sin(x[2])]])).reshape(1, 2)
    return -c * L_direct.T / np.linalg.norm(L_direct)


def du_bar(x, y, grad, c):
    p = 10
    la = np.array([[1, 0], [0, 1 / np.sqrt(p)]])
    return -c * la @ la @ L_Gh(x, y, grad).T / np.linalg.norm(L_Gh(x, y, grad) @ la)


def du_bar_new(x, rdrx, c):
    p = 100
    la = np.array([[1, 0], [0, 1 / np.sqrt(p)]])
    return -c * la @ la @ L_Gh_new(x, rdrx).T / np.linalg.norm(L_Gh_new(x, rdrx) @ la)


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


# def get_transformation(x: tensor) -> tensor:
#     cos_term = torch.cos(x[2])
#     sin_term = torch.sin(x[2])
#     # transformation = torch.zeros((3, 3), requires_grad=False)
#     # transformation[0, :] = tensor([cos_term, - sin_term, x[0]])
#     # transformation[1, :] = tensor([sin_term, cos_term, x[1]])
#     # transformation[2, 2] = 1
#     return tensor([[cos_term, - sin_term, x[0]],
#                    [sin_term, cos_term, x[1]],
#                    [0, 0, 1]], requires_grad=True)


# def phi(SDF: tensor, kappa: float) -> tensor:
#     return 0.5 * (1 + torch.erf(SDF / (2 ** 0.5 * kappa) - 2))


def load_local_traj(path):
    data = np.load(path)['arr_0']
    return data


def sock_init(ip, port, mode):
    sock = socket.socket(socket.AF_INET,  # Internet
                         socket.SOCK_DGRAM)  # UDP
    if mode == 'r': sock.bind((ip, port))
    return sock


def sock_read(sock):
    data, addr = sock.recvfrom(4096)  # buffer size is 1024 bytes
    if not data: return -1
    data = pickle.loads(data)
    return data


def sock_write(sock, ip, port, data):
    data_string = pickle.dumps(data)
    sock.sendto(data_string, (ip, port))
    return 0


def _drawTriGrad():
    # import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    pos = [(0.5 * i - 0.5, 0.5 * j + 0.5) for i in range(-30, 31, 1) for j in range(-30, 31, 1)]

    result = []
    plot = plt.figure(1, (8, 8))
    plt.figure(1, (8, 8))
    ax = plot.gca()
    vertexAgent1 = np.array([0, 0])
    vertexAgent2 = VisionRange * np.array([np.cos(0.5 * ViewAngle), np.sin(-0.5 * ViewAngle)])
    vertexAgent3 = VisionRange * np.array([np.cos(0.5 * ViewAngle), np.sin(0.5 * ViewAngle)])
    FOVAgent = Polygon([vertexAgent1, vertexAgent2, vertexAgent3], alpha=0.7)
    ax.add_patch(FOVAgent)
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    c1 = [(np.cos(t * (np.pi - ViewAngle) / 10 + 0.5 * (np.pi + ViewAngle)),
           np.sin(t * (np.pi - ViewAngle) / 10 + 0.5 * (np.pi + ViewAngle)))
          for t in range(11)]
    c2 = [
        (np.cos(t * 0.5 * (np.pi + ViewAngle) / 10) + vertexAgent3[0],
         np.sin(t * 0.5 * (np.pi + ViewAngle) / 10) + vertexAgent3[1])
        for t in range(11)]
    c3 = [(np.cos(t * 0.5 * (np.pi + ViewAngle) / 10) + vertexAgent2[0],
           np.sin(-t * 0.5 * (np.pi + ViewAngle) / 10) + vertexAgent2[1])
          for t in range(11)]
    l1 = [((c1[0][0] - c2[-1][0]) * t / 10 + c2[-1][0], (c1[0][1] - c2[-1][1]) * t / 10 + c2[-1][1]) for t in range(10)]
    l2 = [((c2[0][0] - c3[0][0]) * t / 10 + c3[0][0], (c2[0][1] - c3[0][1]) * t / 10 + c3[0][1]) for t in range(10)]
    l3 = [((c3[-1][0] - c1[-1][0]) * t / 10 + c1[-1][0], (c3[-1][1] - c1[-1][1]) * t / 10 + c1[-1][1]) for t in
          range(10)]
    # pos = c1 + c2 + c3+l1+l2+l3
    for p in pos:
        s, g = triangle_SDF(p, ViewAngle, VisionRange)
        plt.arrow(p[0], p[1], g[0], g[1], head_width=0.2)
        # plt.scatter(p[0],p[1],c=[])
    plt.show()


def _drawPolyGrad():
    resolution = 0.01
    xlim, ylim = 10, 10
    poly = np.array([[4.15058908, 1.8007605],
                     [2.6075482, -7.],
                     [2., -7.],
                     [1.85917585, -7.57628843],
                     [-2.2330149, -4.14926839],
                     [-1., -3.],
                     [0.36793827, 0.41984566],
                     [4.15058908, 1.8007605]])
    pos = np.array([[resolution * i - 0.5, resolution * j + 0.5]
                    for i in range(-int(xlim / resolution), int(xlim / resolution) + 1, 1)
                    for j in range(-int(ylim / resolution), int(ylim / resolution) + 1, 1)])
    res = np.zeros((len(pos), 1))
    color = np.zeros((len(pos), 3))
    plot = plt.figure(1, (8, 8))
    plt.figure(1, (8, 8))
    ax = plot.gca()
    ax.set_xlim(-xlim, xlim)
    ax.set_ylim(-ylim, ylim)
    for i, p in enumerate(pos):
        res[i] = polygon_SDF(poly, p)
    upper = np.max(res)
    lower = np.min(res)
    for i in range(len(pos)):
        if abs(np.mod(res[i], 1)) < 0.01:
            color[i, :] = np.array([1, 1, 1])
            continue
        if res[i] >= 0:
            color[i, 2] = res[i] / upper
        else:
            color[i, 0] = res[i] / lower
    plt.scatter(pos[:, 0], pos[:, 1], c=color)
    plt.plot(poly[:, 0], poly[:, 1])
    plt.show()


def _drawAnalyHM():
    import matplotlib.pyplot as plt
    cam = np.array([0, 0, 0])
    tgpos = np.array([[i * 0.5, j * 0.5, 0] for i in range(-40, 40) for j in range(-40, 40)])
    res = []
    theta = cam[2]
    for pos in tgpos:
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        _, g = triangle_SDF(R @ (pos - cam)[0:2], np.pi / 3, 10)
        res.append((g @ dqxdx(cam, pos))[2])
    res = (np.array(res)).reshape(80, 80)
    plt.imshow(res, cmap='hot', interpolation='nearest')
    plt.show()


if __name__ == '__main__':
    # _drawTriGrad()
    # _drawPolyGrad()
    _drawAnalyHM()
