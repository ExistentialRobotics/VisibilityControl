# target_publisher/target_publisher_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.qos import QoSProfile
import numpy as np

class TargetPublisherNode(Node):
    def __init__(self):
        super().__init__('target_publisher_node')
        qos_profile = QoSProfile(depth=10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/carla/target/control/set_transform', qos_profile)
        self.velocity_publisher = self.create_publisher(Twist, '/target_velocity', qos_profile)
        self.declare_parameter('trajectory_file', 'config/Lsj.npz')
        self.trajectory_file = self.get_parameter('trajectory_file').get_parameter_value().string_value
        
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.trajectory_data = self.load_trajectory(self.trajectory_file)
        self.index = 0

    def load_trajectory(self, file_path):
        try:
            data = np.load(file_path)
            trajectory = data['arr_0']
            self.get_logger().info(f'Loaded trajectory data from {file_path}')
            return trajectory
        except Exception as e:
            self.get_logger().error(f'Failed to load trajectory file: {e}')
            return np.array([])

    def timer_callback(self):
        if self.index < len(self.trajectory_data):
            row = self.trajectory_data[self.index]
            
            # Publish pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = row[0]
            pose_msg.pose.position.y = row[1]
            pose_msg.pose.position.z = 0.0
            # Convert theta to quaternion
            q = self.euler_to_quaternion(0, 0, row[2])
            pose_msg.pose.orientation.x = q[0]
            pose_msg.pose.orientation.y = q[1]
            pose_msg.pose.orientation.z = q[2]
            pose_msg.pose.orientation.w = q[3]
            self.pose_publisher.publish(pose_msg)

            # Publish velocity
            velocity_msg = Twist()
            velocity_msg.linear.x = row[3]
            velocity_msg.linear.y = row[4]
            velocity_msg.linear.z = 0.0
            velocity_msg.angular.z = row[5]
            self.velocity_publisher.publish(velocity_msg)

            self.index += 1
            self.index = self.index % len(self.trajectory_data)
        else:
            self.get_logger().info('Finished publishing all trajectory points.')
            self.timer.cancel()

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion.
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

