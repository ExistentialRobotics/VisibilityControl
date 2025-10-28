#!/usr/bin/env python3

import carla
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from math import cos, sin


class CarlaVehicleSpawner(Node):
    def __init__(self, trajectory_file):
        super().__init__('carla_vehicle_spawner')
        self.pose_publisher = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.arrow_publisher = self.create_publisher(PoseStamped, '/target_arrow', 10)

        self.robot_reset = self.create_publisher(PoseStamped, '/carla/ego_vehicle/control/set_transform', 10)

        # Load the trajectory from the npz file
        self.trajectory = np.load(trajectory_file)['arr_0']
        self.trajectory_index = 0

        # Connect to Carla
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        # Get the blueprint of the vehicle
        self.blueprint_library = self.world.get_blueprint_library()
        self.vehicle_bp = self.blueprint_library.filter('vehicle.ford.mustang')[0]  # Spawn any available vehicle

        # Spawn the vehicle
        self.spawn_point = self.get_spawn_point()
        self.vehicle = self.world.spawn_actor(self.vehicle_bp, self.spawn_point)

        self.timer = self.create_timer(0.04, self.update_vehicle_pose)


    def get_spawn_point(self):
        spawn_transform = carla.Transform()
        spawn_transform.location = carla.Location(x=self.trajectory[0, 0], y=self.trajectory[0, 1], z=0.2)
        spawn_transform.rotation.yaw = np.degrees(self.trajectory[0, 2])
        return spawn_transform

    def update_vehicle_pose(self):
        if self.trajectory_index < len(self.trajectory):
            x, y, theta, xdot, ydot, theta_dot = self.trajectory[self.trajectory_index]

            # Update vehicle position and orientation
            transform = self.vehicle.get_transform()
            transform.location.x = x
            transform.location.y = -y
            transform.rotation.yaw = -np.degrees(theta)
            self.vehicle.set_transform(transform)

            # Publish the pose to ROS2
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = 0.0  # Assume 2D ground plane
            pose_msg.pose.orientation.z = sin(theta / 2.0)
            pose_msg.pose.orientation.w = cos(theta / 2.0)
            self.pose_publisher.publish(pose_msg)
            
            # Publish the pose to ROS2
            arrow_msg = PoseStamped()
            arrow_msg.header.stamp = self.get_clock().now().to_msg()
            arrow_msg.header.frame_id = 'map'
            arrow_msg.pose.position.x = x
            arrow_msg.pose.position.y = y
            arrow_msg.pose.position.z = 20.0  # Assume 2D ground plane
            arrow_msg.pose.orientation.y = 0.7071068
            arrow_msg.pose.orientation.w = 0.7071068
            self.arrow_publisher.publish(arrow_msg)

            self.trajectory_index += 1
        else:
            self.trajectory_index = 0


def main(args=None):
    # Initialize ROS2 node
    rclpy.init(args=args)

    # Load trajectory and run the vehicle spawner
    trajectory_file = '/home/joe/Documents/Bringups/carla-ros-cbf/carla_ros_cbf/robot/planner/Lsj.npz'  # Replace with your npz file path
    vehicle_spawner = CarlaVehicleSpawner(trajectory_file)

    try:
        rclpy.spin(vehicle_spawner)
    except KeyboardInterrupt:
        pass

    vehicle_spawner.vehicle.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
