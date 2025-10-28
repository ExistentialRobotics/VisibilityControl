import heapq
import signal
import time

from visualization_msgs.msg import Marker

from env.simple_env import Environment
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, Twist, Vector3
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2
from cv_bridge import CvBridge
from planner_main import PlannerROS
from env.config import *
# from robot.controller.basic import BasicController
from robot.controller.basic_cvx import BasicController
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from utils_planner import SDF_RT
import pickle
import csv
import carla


# import torch

def heuristic(a, b):
    # Using Manhattan distance as the heuristic
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def find_nearest_255_a_star(image, start_point):
    rows, cols = image.shape
    start_x, start_y = start_point

    # Priority queue for the A* algorithm
    pq = [(0, start_x, start_y)]
    visited = set()
    heapq.heapify(pq)

    # Define the possible movements (8-directional)
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)]

    while pq:
        current_distance, x, y = heapq.heappop(pq)

        # Check if the new point is within the image bounds
        if 0 <= x < rows and 0 <= y < cols:
            if image[x, y] == 255:
                return (x, y)

        visited.add((x, y))

        # Check all 8 directions
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if (nx, ny) not in visited:
                distance = current_distance + 1
                heapq.heappush(pq, (distance + heuristic((nx, ny), (start_x, start_y)), nx, ny))
                visited.add((nx, ny))

    return None  # Return None if no 255 pixel is found


class MapScanController(Node):
    def __init__(self):
        super().__init__('map_scan_controller')

        self.subscription_map = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.subscription_target = self.create_subscription(
            PoseStamped, '/target_pose', self.target_callback, 10)
        self.subscription_pose = self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.pose_callback, 10)
        self.subscription_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.subscriber_control = self.create_subscription(Twist, '/ref_vel', self.ref_vel_callback, 10)
        self.subscriber_pose_gt = self.create_subscription(Odometry, '/ego_vehicle/odometry', self.odom_callback, 10)

        self.publisher_control = self.create_publisher(Twist, '/carla/ego_vehicle/control/set_target_velocity', 10)
        self.publisher_fov = self.create_publisher(Marker, '/fov', 10)
        self.publisher_rbt_traj = self.create_publisher(Marker, '/rbt', 10)
        self.publisher_tgt_traj = self.create_publisher(Marker, '/tgt', 10)

        signal.signal(signal.SIGINT, self.signal_handler)

        self.bridge = CvBridge()

        self.map = None
        self.target = None
        self.pose = None
        self.pose_gt = None
        self.scan = None

        self.env = Environment(agent=None, horizon=horizon, tau=tau, psi=psi, radius=radius, epsilon_s=epsilon_s, obs=None, wls=None)
        # self.cbf = BasicController(self.env)
        self.cbf_cvx = BasicController()
        self.planner = PlannerROS()
        self.plan = False
        self.ref_u = np.array([[18], [0]])
        self.u_prev = np.array([[18], [0]])
        self.counter = 0
        ref = np.array([[18], [0]])
        ref_mpt = None
        self.planner_cd = 20
        self.is_target_seen = False
        self.target_vel = np.array([[0], [0]])
        self.target_prev = np.array([[0], [0], [0]])
        self.target_timestamp = 0.0
        self.sdf_record = []
        self.obs_record = []
        self.cHz = []
        self.time_record = []
        self.rbt_record = []
        self.tgt_record = []
        self.u_record = []

        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)

    def odom_callback(self, msg):
        # Extract position data (x, y, z)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Extract orientation data (quaternion) and convert it to Euler angles
        orientation_q = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion(orientation_q)
        self.pose_gt = (x, y, yaw)

    def map_callback(self, msg):
        # Convert OccupancyGrid to image
        width = msg.info.width
        height = msg.info.height
        map_data = np.array(msg.data).reshape((height, width))
        map_image = np.zeros((height, width), dtype=np.uint8)
        map_image[map_data == 0] = 255  # Free space
        map_image[map_data == -1] = 255  # Unknown space
        map_image[map_data > 0] = 0  # Occupied space
        map_origin = msg.info.origin.position
        self.map_info = msg.info
        self.map_origin = (map_origin.x, map_origin.y, map_origin.z)
        self.map = map_image[::-1]

    def target_callback(self, msg):
        ang = euler_from_quaternion((msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
        self.target = (msg.pose.position.x, msg.pose.position.y, ang[2])

    def pose_callback(self, msg):
        ang = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, ang[2])

    def scan_callback(self, msg):
        # Convert LaserScan to points
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        if self.pose:
            pose_x, pose_y, pose_yaw = self.pose
            angles = angle_min + np.arange(len(ranges)) * angle_increment + pose_yaw
            points = np.vstack((ranges * np.cos(angles), ranges * np.sin(angles))).T
            points[:, 0] += pose_x
            points[:, 1] += pose_y
            self.scan = points
            self.process_data()

    def ref_vel_callback(self, msg):
        self.ref_u = np.array([[msg.linear.x], [msg.angular.z]])

    def process_data(self):
        print("running")
        if self.map is not None and self.target is not None and self.pose is not None and self.scan is not None:
            if not self.is_target_seen:
                self.is_target_seen = True
                self.target_prev = self.target
                self.target_timestamp = self.get_clock().now().nanoseconds / 1e9
                return
            pose_x, pose_y, _ = self.pose
            now = self.get_clock().now().nanoseconds / 1e9
            self.target_vel = self.calculate_velocity(self.target, self.target_prev, now - self.target_timestamp)
            self.target_prev = self.target
            self.target_timestamp = self.get_clock().now().nanoseconds / 1e9
            self.env._global_map = self.map
            robot_m = self.w2m(self.map_info, np.array(self.pose))
            visibility_m = self.env.SDF_RT_circular(robot_m, 50, 300, self.map)
            visibility_m = np.hstack((visibility_m, np.zeros((len(visibility_m), 1))))
            visibility_w = (self.m2w(self.map_info, visibility_m.T)[0:2, :]).T[1:-1]
            time_init = time.time()
            u, fov, h, d_obs = self.cbf_cvx.solvecvx(np.array(self.pose),
                                                     np.array(self.target).reshape((3, 1)),
                                                     np.array(self.target_vel).reshape((3, 1)),
                                                     self.ref_u,
                                                     visibility_w,
                                                     self.map_info,
                                                     self.map,
                                                     self.u_prev)
            time_end = time.time()

            # new_path = Path()
            # new_path.header.frame_id = 'map'
            # new_path.header.stamp = self.get_clock().now().to_msg()
            # if ref_mpt is None: return
            # print("no path found")
            # for point in path:
            #     pose = PoseStamped()
            #     pose.header = new_path.header
            #     pose.pose.position.x = point[0]
            #     pose.pose.position.y = point[1]
            #     pose.pose.position.z = 0.0  # Assuming z is 0 for a 2D path
            #
            #     # Convert Euler angle (theta) to quaternion
            #     quaternion = quaternion_from_euler(0, 0, point[2])
            #     pose.pose.orientation.x = quaternion[0]
            #     pose.pose.orientation.y = quaternion[1]
            #     pose.pose.orientation.z = quaternion[2]
            #     pose.pose.orientation.w = quaternion[3]
            #
            #     new_path.poses.append(pose)
            # self.publisher_path.publish(new_path)

            if u is not None:
                u = u.flatten()
                u_c = Twist()
                u_c.linear = Vector3(x=float(u[0]))
                u_c.angular = Vector3(z=-float(u[1]))
                self.publisher_control.publish(u_c)
            self.sdf_record.append(h)
            self.cHz.append(1 / (time_end - time_init))
            self.obs_record.append(d_obs)
            self.time_record.append(time_end)
            self.rbt_record.append(list(self.pose))
            self.tgt_record.append(list(self.target))
            self.u_record.append(u.flatten().tolist())
            self.publish_2d_shape(fov, self.publisher_fov, (1.0, 1., 0., 0., 1.))
            self.publish_2d_shape(np.array(self.tgt_record), self.publisher_tgt_traj, (2.0, 0., 0.5, 1.0, 1.), closed=False)
            self.publish_2d_shape(np.array(self.rbt_record), self.publisher_rbt_traj, (2.0, 1., 0., 0., 0.7), closed=False)
            # self.visualize_path_only(self.client, np.array(self.tgt_record))
            # self.visualize_path_only(self.client, np.array(self.rbt_record))

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
        x_m = x_m.reshape((3, -1))
        m_w = np.array([[map_info.origin.position.x], [map_info.origin.position.y], [0]])
        scale = np.array([[res, 0, 0], [0, res, 0], [0, 0, 1]])
        mRd = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        mtd = np.array([[-h], [0], [-np.pi / 2]])
        return scale @ mRd @ (x_m + mtd) + m_w

    @staticmethod
    def calculate_velocity(pose1, pose2, tau):
        """
        Calculate velocity given two SE(2) poses and time period tau.

        pose1: tuple (x1, y1, theta1)
        pose2: tuple (x2, y2, theta2)
        tau: time interval between the two poses
        """
        # Unpack poses
        x1, y1, theta1 = pose1
        x2, y2, theta2 = pose2

        # Calculate linear velocities
        v_x = (x2 - x1) / tau
        v_y = (y2 - y1) / tau

        # Calculate angular velocity
        delta_theta = np.arctan2(np.sin(theta2 - theta1), np.cos(theta2 - theta1))  # Shortest angle difference
        omega = delta_theta / tau

        return np.array([v_x, v_y, omega])

    def publish_2d_shape(self, positions: np.ndarray, publisher, line: tuple, closed=True):
        # Create a publisher for Marker messages
        # Initialize the Marker message
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "shape"
        marker.id = 0
        marker.type = Marker.LINE_STRIP  # LINE_STRIP to connect points
        marker.action = Marker.ADD

        # Set marker parameters (line width and color)
        marker.scale.x = line[0]  # Line width
        marker.color.r = line[1]
        marker.color.g = line[2]
        marker.color.b = line[3]
        marker.color.a = line[4]

        # Convert positions to Point and add to marker points
        for pos in positions:
            point = Point()
            point.x = pos[0]
            point.y = pos[1]
            point.z = 0.0  # For 2D shapes, z is 0
            marker.points.append(point)

        # Optionally close the shape by connecting the last point to the first
        if closed:
            closing_point = Point()
            closing_point.x = positions[0][0]
            closing_point.y = positions[0][1]
            closing_point.z = 0.0
            marker.points.append(closing_point)

        # Publish the marker
        publisher.publish(marker)

    def data_storage(self):
        my_data = [self.time_record, self.sdf_record, self.cHz, self.obs_record, self.rbt_record, self.tgt_record]
        with open('0.pkl', 'wb') as file:
            pickle.dump(my_data, file)
        # with open('CARLA_data_everything_updated.csv', 'w', newline='') as file:
        #     writer = csv.writer(file)
        #     writer.writerows(my_data)

    def signal_handler(self, sig, frame):
        # Call the function to save the data
        self.data_storage()

        # Gracefully shutdown the node
        self.get_logger().info('Shutting down...')
        self.destroy_node()
        rclpy.shutdown()

    def visualize_path_only(self, client, trajectory, z_height=0.1):
        """
        Visualizes a trajectory in CARLA without a vehicle.

        Args:
            client (carla.Client): The CARLA client.
            trajectory (numpy.ndarray): The trajectory points as an n*2 NumPy array.
            z_height (float): The height at which to place the trajectory points.
        """
        world = client.get_world()

        # Convert NumPy array to CARLA locations
        locations = [carla.Location(x=float(point[0]), y=-float(point[1]), z=z_height) for point in trajectory]

        # Draw the trajectory in the environment
        for i in range(len(locations) - 1):
            start = locations[i]
            end = locations[i + 1]
            world.debug.draw_line(start, end, thickness=0.1, color=carla.Color(255, 0, 0), life_time=120.0)


def main(args=None):
    rclpy.init(args=args)
    node = MapScanController()
    try:
        # Run the node until Ctrl+C is pressed
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure clean shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
