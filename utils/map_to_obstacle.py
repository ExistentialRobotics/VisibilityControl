import cv2
import matplotlib.pyplot as plt
import numpy as np

from env.config import *


def display2map_vec(map_origin, ratio, x_d):
    """Takes in nx2 array of vertices in pixel frame and converts to nx3 world frame vertices"""
    x_d = (np.array([[1, 0], [0, 1], [0, 0]]) @ x_d.T).T # nx3 array
    x_d_shift = x_d - map_origin
    d2m = np.array([[0, 1 / ratio, 0],[-1 / ratio, 0, 0],[0, 0, 1]])
    return (d2m @ x_d_shift.T).T # dot product of d2m with nx3 array

def is_duplicate_obstacle(cand_obs, obstacles: list) -> bool:
    for other_obs in obstacles:
        dists = []
        for point in cand_obs:
            dists.append(np.min(np.linalg.norm(other_obs - point, axis=1)))
        if np.array(dists).sum() < 5:
            return True
    return False

# openCV needs 0s to represent the shapes to find contours for so flip the original carla map
obstacles_file_path = "/home/minnan/Documents/Seek/data/obstacles_carla.npy"
map = cv2.imread(map_file_path)
# get rid of boundary on the outside of the map
map_reduced = map[150:830,150:826]
# cv2.imwrite("/home/minnan/Documents/Seek/map/carla_map_flipped_reduced.png",map_reduced)

# set new map height and width, but keep ratio the same
MAP_HEIGHT = map_reduced.shape[0]
MAP_WIDTH = map_reduced.shape[1]
map_ratio = 2
map_origin = np.array([MAP_HEIGHT // 2, MAP_WIDTH // 2, np.pi / 2])

# run canny edge detection before passing into findContours
edge = cv2.Canny(map_reduced, 0, 255)
# cv2.imwrite("/home/minnan/Documents/Seek/map/carla_map_edge_flipped_reduced.png", edge)

obstacles = []
# find contours, only keep vertices rather than each point on boundary; return all contours
contours, hierarchy = cv2.findContours(edge, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
# each contour is an obstacle (currently in pixel frame)
for contour in contours:
    contour = contour[:,0,:]
    contour_world = display2map_vec(map_origin, map_ratio, contour)
    contour_world[:, [0, 1]] = contour_world[:, [1, 0]]
    contour_world *= -1 # output from pixel to world projection matrix has signs flipped in x,y
    if is_duplicate_obstacle(contour_world, obstacles):
        pass # don't add the obstacle to the list if it's a duplicate
    else:
        obstacles.append(contour_world)

np.save(obstacles_file_path, np.array(obstacles, dtype=object), allow_pickle=True)

# check that the extracted obstacle vertices make sense
for obstacle in obstacles:
    plt.scatter(obstacle[:,0],obstacle[:,1])
plt.savefig("/home/minnan/Documents/Seek/map/carla_map_obstacle_vertices_extracted.png")