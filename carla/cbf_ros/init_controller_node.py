from robot.controller.controller import MyControllerNode

def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = MyControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
