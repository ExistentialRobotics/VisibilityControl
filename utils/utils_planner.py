from functools import lru_cache

import numpy as np

RAY_TRACING_DEBUG = False
DEMO = False
SHOW_FPS = False
global_map = np.zeros((100, 100))


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


def boundaries(robot_pose, fov_angle, distance):
    x0, y0, theta = robot_pose.squeeze()
    x1 = x0 + distance * np.cos(theta - 0.5 * fov_angle)
    y1 = y0 + distance * np.sin(theta - 0.5 * fov_angle)
    x2 = x0 + distance * np.cos(theta + 0.5 * fov_angle)
    y2 = y0 + distance * np.sin(theta + 0.5 * fov_angle)
    return [x1, y1], [x2, y2]


def SDF_RT(robot_pose, fov, radius, RT_res, grid, inner_r=10):
    global global_map
    global_map = grid
    pts = raytracing(robot_pose, fov, radius, RT_res)
    in_1, in_2 = boundaries(robot_pose, fov, inner_r)
    if inner_r == 0:
        pts = [in_1] + pts + [in_2]
    else:
        pts = [in_1] + pts + [in_2, in_1]
    return vertices_filter(np.array(pts))


def raytracing(robot_pose, fov, radius, RT_res):
    x0, y0, theta = robot_pose
    out_1, out_2 = boundaries(robot_pose, fov, radius)
    # y_mid = [y0 + radius * np.sin(theta - 0.5 * fov + i*fov / RT_res) for i in range(RT_res+1)]
    # x_mid = [x0 + radius * np.cos(theta - 0.5 * fov + i*fov / RT_res) for i in range(RT_res+1)]
    y_mid = np.linspace(out_1[1], out_2[1], RT_res)
    x_mid = np.linspace(out_1[0], out_2[0], RT_res)
    pts = []
    for i in range(len(x_mid)):
        xx, yy = DDA(int(x0), int(y0), int(x_mid[i]), int(y_mid[i]))
        if not pts or (yy != pts[-1][1] or xx != pts[-1][0]):
            pts.append([xx, yy])
    return pts


def vertices_filter(polygon, angle_threshold=0.05):
    diff = polygon[1:] - polygon[:-1]
    diff_norm = np.sqrt(np.einsum('ij,ji->i', diff, diff.T))
    unit_vector = np.divide(diff, diff_norm[:, None], out=np.zeros_like(diff), where=diff_norm[:, None] != 0)
    angle_distance = np.round(np.einsum('ij,ji->i', unit_vector[:-1, :], unit_vector[1:, :].T), 5)
    angle_abs = np.abs(np.arccos(angle_distance))
    minimum_polygon = polygon[[True] + list(angle_abs > angle_threshold) + [True], :]
    return minimum_polygon


def polygon_SDF(polygon, point):
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


def boundaries(robot_pose, fov_angle, distance):
    x0, y0, theta = robot_pose.squeeze()
    x1 = x0 + distance * np.cos(theta - 0.5 * fov_angle)
    y1 = y0 + distance * np.sin(theta - 0.5 * fov_angle)
    x2 = x0 + distance * np.cos(theta + 0.5 * fov_angle)
    y2 = y0 + distance * np.sin(theta + 0.5 * fov_angle)
    return [x1, y1], [x2, y2]


@lru_cache(None)
def DDA(x0, y0, x1, y1):
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
        if 0 < int(x) < len(global_map) and 0 < int(y) < len(global_map[0]):
            if global_map[int(x), int(y)] == 0:
                break
        x = x + x_inc
        y = y + y_inc
    return int(x) + 1, int(y) + 1


def normalize_angle(angle):
    """Normalize an angle to be within the range [-π, π]"""
    warp_angle = (angle + np.pi) % (2 * np.pi) - np.pi
    # Handle the case when angle = -π so that the result is -π, not π
    if warp_angle == -np.pi and angle > 0:
        return np.pi
    return warp_angle


# try:
#     from ompl import base as ob
#     from ompl import geometric as og
#     from ompl import util as ou
# except ImportError:
#     raise ImportError("Container does not have OMPL installed")
# import skimage.morphology as skim


def geom2pix(pos, res=0.05, size=(480, 480)):
    """
    Convert geometrical position to pixel co-ordinates. The origin
    is assumed to be at [image_size[0]-1, 0].
    :param pos: The (x,y) geometric co-ordinates.
    :param res: The distance represented by each pixel.
    :param size: The size of the map image
    :returns (int, int): The associated pixel co-ordinates.
    NOTE: The Pixel co-ordinates are represented as follows:
    (0,0)------ X ----------->|
    |                         |
    |                         |
    |                         |
    |                         |
    Y                         |
    |                         |
    |                         |
    v                         |
    ---------------------------
    """
    return np.int16(np.floor(pos[0] / res)), np.int16(size[0] - 1 - np.floor(pos[1] / res))


# class ValidityChecker(ob.StateValidityChecker):
#     '''A class to check if an obstacle is in collision or not.
#     '''
#
#     def __init__(self, si, CurMap, MapMask=None, res=0.05, robot_radius=0.1):
#         '''
#         Intialize the class object, with the current map and mask generated
#         from the transformer model.
#         :param si: an object of type ompl.base.SpaceInformation
#         :param CurMap: A np.array with the current map.
#         :param MapMask: Areas of the map to be masked.
#         '''
#         super().__init__(si)
#         self.size = CurMap.shape
#         # Dilate image for collision checking
#         InvertMap = np.abs(1 - CurMap)
#         InvertMapDilate = skim.dilation(InvertMap, skim.disk(robot_radius / res))
#         MapDilate = abs(1 - InvertMapDilate)
#         if MapMask is None:
#             self.MaskMapDilate = MapDilate > 0.5
#         else:
#             self.MaskMapDilate = np.logical_and(MapDilate, MapMask)
#
#     def isValid(self, state):
#         '''
#         Check if the given state is valid.
#         :param state: An ob.State object to be checked.
#         :returns bool: True if the state is valid.
#         '''
#         pix_dim = geom2pix(state, size=self.size)
#         return self.MaskMapDilate[pix_dim[1], pix_dim[0]]
