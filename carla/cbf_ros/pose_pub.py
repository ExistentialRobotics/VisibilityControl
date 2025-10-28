import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


class TFToPosePublisher(Node):
    def __init__(self):
        super().__init__('tf_to_pose_publisher')

        # Create a publisher for the PoseStamped message
        self.pose_publisher = self.create_publisher(PoseStamped, 'pose_carla', 10)

        # Create a tf2 Buffer and Listener to get the transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to regularly query the transform and publish the pose
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Specify the source and target frames
        from_frame = 'map'  # Change to the appropriate source frame
        to_frame = 'ego_vehicle'  # Change to the appropriate target frame

        try:
            # Lookup the transform from the tf tree
            transform = self.tf_buffer.lookup_transform(from_frame, to_frame, rclpy.time.Time())

            # Extract translation and rotation (quaternion)
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            # Create PoseStamped message
            pose_msg = PoseStamped()

            # Set the header with the timestamp from the transform
            pose_msg.header.stamp = transform.header.stamp
            pose_msg.header.frame_id = from_frame  # Use the source frame as the frame_id

            # Set the position from the transform
            pose_msg.pose.position.x = translation.x
            pose_msg.pose.position.y = translation.y
            pose_msg.pose.position.z = translation.z

            # Set orientation (rotation) from the transform
            pose_msg.pose.orientation = rotation

            # Publish the PoseStamped message
            self.pose_publisher.publish(pose_msg)

        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TFToPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
