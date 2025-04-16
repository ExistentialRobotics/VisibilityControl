import cv2
import matplotlib.pyplot as plt
import numpy as np

from utils.utils import map2display

MAP_HEIGHT = 980
MAP_WIDTH = 976

obstacles_file_path = "../data/obstacles.npz"
map_file_path = "../env/map/custom_map_1.png"

# create some obstacles, where each entry is a set of vertices - theta coordinate set to 0 as it's not used
obstacles = [
    np.array([[-20,-20,0],[-20,20,0],[20,20,0],[20,-20,0],[-20,-20,0]]),
    # np.array([[-50,-50,0],[-50,-30,0],[-30,-30,0],[-30,-50,0]])
]

np.savez(obstacles_file_path, obstacles)
obstacles = np.load(obstacles_file_path)['arr_0']

# create, save and read in the map using openCV
_map = np.zeros((MAP_HEIGHT, MAP_WIDTH), dtype=np.uint8)
map_ratio = 2
map_origin = np.array([[MAP_HEIGHT // 2], [MAP_WIDTH // 2], [np.pi / 2]])

for obstacle in obstacles:
    # project the world frame obstacle vertex coordinates to the pixel frame
    obs_disp = np.array([map2display(map_origin, map_ratio, vertex.reshape(3,1)) for vertex in obstacle])
    obs_disp = obs_disp[:,0:2].reshape((-1,1,2)).astype(np.int32)
    # fill in the polygon given the vertices
    cv2.fillPoly(_map, [obs_disp], color=(255,0,0))

# flip the map convention for planner
# _map[_map==0] = 2
# _map[_map==255] = 0
# _map[_map==2] = 255
cv2.imwrite(map_file_path, _map.astype(np.uint8))
plt.imshow(_map)
plt.show()