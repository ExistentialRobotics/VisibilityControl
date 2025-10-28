import numpy as np

import rclpy
# from env.simple_env import Environment
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
import tf2_ros
from robot.controller.basic import BasicController
from robot.controller.mpc.drc_controller import DRCController
from robot.estimator.observer import AugmentedEKF
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from env.config import *
from tf_transformations import euler_from_quaternion


class MyControllerNode(Node):
    def __init__(self):
        super().__init__('my_controller_node')

        # Subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        # scan callback is where the call to the qp solver is made
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # uncomment this to test with ros without tf tree
        # self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/pose', self.pose_callback, 10)
        self.ref_vel_sub = self.create_subscription(Twist, '/ref_vel', self.ref_vel_callback, 10)
        self.predicted_tgt_pub = self.create_publisher(PoseStamped, '/pred_tgt_pose', 10)

        # create tf listener and timer to call the pose callback
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.timer_period = 0.2  # seconds
        self.predict_step_timer = 0.01
        self.target_pose_timer = self.create_timer(self.timer_period, self.target_pose_callback)
        self.pose_timer = self.create_timer(self.timer_period, self.pose_callback)
        self.target_state_est_timer = self.create_timer(self.predict_step_timer, self.target_state_est_callback)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize variables to store the received data
        self.map = None
        self.map_info = None
        self.scan = None
        self.target_pose = None
        self.prev_target_pose = None  # for calculating target velocity
        self.target_state = None
        self.prev_target_pose_ts = None  # prev target pose timestamp to compute velocity
        self.target_velocity = np.array([0.,0.]) # default value until its updated
        self.pose = None
        self.ref_vel = None  # the optimal control to output to the turtlebot motor
        # whether to use visibility constraint as a sample in drc formulation
        self.use_sampled_cbf_visibility = False
        # if use_sampled_cbf_visibility is true, this must be true
        self.use_raytracing = True
        # Timer to update the control loop
        self.fov_calc_ongoing = False
        # self.env = Environment(agent=None, horizon=horizon, tau=tau, psi=psi, radius=radius, epsilon_s=epsilon_s, obs=None, wls=None)
        # self.solver = BasicController(self.env)
        self.solver = DRCController()
        #### Target state estimation #####
        # The observer knows initial position but not velocity
        self.initial_target_cov = np.diag([0, 0, 0, 0, 0])  # TODO: how do we initialize the target covariance?
        self.process_noise = np.diag([0, 0, 0, 1, 1])  # high variance to account for inaccurate constant velocity model
        self.observation_noise = np.diag([0.05, 0.05, 0.05, 0.1, 0.1])  # AprilTag detection variance for (x,y, theta, u, v)

        self.estimator = AugmentedEKF(
            initial_covariance=self.initial_target_cov,
            process_noise_cov=self.process_noise,
            observation_noise_cov=self.observation_noise
        )

        self.solver_ongoing = False

    def map_callback(self, msg):
        # self.get_logger().info('Map received.')
        if self.use_raytracing:
            width = msg.info.width
            height = msg.info.height
            map_data = np.array(msg.data).reshape((height, width))
            map_image = np.zeros((height, width), dtype=np.uint8)
            map_image[map_data == 0] = 255  # Free space
            map_image[map_data == -1] = 255  # Unknown space
            map_image[map_data > 0] = 0  # Occupied space
            # create dict with map and metadata to pass to controller
            self.map_info = {
                "height":msg.info.height, 
                "resolution":msg.info.resolution, 
                "origin":np.array((msg.info.origin.position.x, msg.info.origin.position.y, msg.info.origin.position.z)), 
                "map":map_image[::-1]
                }

    def scan_callback(self, msg):
        self.scan = np.array(msg.ranges)
        # self.get_logger().info('Lidar scan received. Calculating obstacle avoidance and target visibility CBF...')
        # simple thread lock to prevent new data being used for ongoing controller calculations
        if not self.solver_ongoing:
            start = self.get_clock().now()
            self.solver_ongoing = True

            pose = self.pose
            target_pose = self.target_pose
            target_velocity = self.target_velocity
            ref_vel = self.ref_vel
            scan = self.scan
            map_info = self.map_info # if not using raytracing, self.map_info is None
            # if map_info is None and self.use_raytracing:
            #     self.get_logger().info('Raytracing mode enabled, but map not available. Cannot find optimal control.')
            #     self.solver_ongoing = False
            #     return # dont let it enter control_loop

            # for debugging: check messages to see if any are None
            # self.get_logger().info("scan" + str(type(scan)))
            # self.get_logger().info("Target pose: " + str(type(target_pose)))
            # self.get_logger().info(str(type(target_velocity)))
            # self.get_logger().info("Agent pose: " + str(type(pose)))
            # self.get_logger().info("Reference input: " + str(type(ref_vel)))
            # self.get_logger().info("Map: " + str(type(map_info["map"])))
            self.control_loop(pose, target_pose, target_velocity, ref_vel, scan, map_info)
            end = self.get_clock().now()
            self.get_logger().info(f'Controller processing time: {(end.nanoseconds-start.nanoseconds)/1e6} ms')
            print("\n")

    def target_pose_callback(self):
        # self.get_logger().info('Target pose received.')
        # get the april tag frame from the tf tree - assume it's named tag_frame
        # print("In the target pose callback")
        try:
            trans = self.tfBuffer.lookup_transform('map', 'tag_frame', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            print(e)
            return

        ang = euler_from_quaternion(
            (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w))
        observed_pose = np.array(
            (trans.transform.translation.x, trans.transform.translation.y, ang[2]))  # turtlebot only yaws so use ang[2]
        observed_pose_ts = trans.header.stamp

        # uncomment this to test with ros without tf tree
        # ang = euler_from_quaternion((msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
        # self.target_pose = np.array((msg.pose.position.x, msg.pose.position.y, ang[2])) # turtlebot only yaws so use ang[2]
        # self.target_pose_ts = msg.header.stamp

        # compute target velocity using cached pose
        if self.prev_target_pose is not None:
            observed_velocity = self.compute_target_velocity(self.prev_target_pose, observed_pose, observed_pose_ts)
        else:
            observed_velocity = self.target_velocity
        # if self.target_pose is not None:
            # print("Distance from predicted to observed position: ", np.linalg.norm(self.target_pose[:2]-observed_pose[:-1]))
        observation = np.concatenate((observed_pose, observed_velocity))
        # pass observation from apriltag to EKF update step. self.target_state is from prediction step (target_state_est_callback)
        if self.target_pose is None:
            self.target_state = self.estimator.update(observation, observation)
        else:
            self.target_state = self.estimator.update(self.target_state, observation)

        # update the previous timestamp and pose with current once velocity has been computed
        self.target_pose = self.target_state[:3]
        self.target_velocity = self.target_state[3:]
        self.prev_target_pose_ts = observed_pose_ts
        self.prev_target_pose = self.target_state[0:3]
        # print("Distance from updated position to observed: ", np.linalg.norm(self.target_pose[:2] - observed_pose[:-1]))

    def pose_callback(self):
        # self.get_logger().info('Self pose received.')
        try:
            # print("Pose callback being executed...")
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            # print(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            print(e)
            return

        ang = euler_from_quaternion(
            (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
             trans.transform.rotation.w))
        self.pose = np.array(
            (trans.transform.translation.x, trans.transform.translation.y, ang[2]))  # turtlebot only yaws so use ang[2]
        # print("Agent pose: ", self.pose)

    def target_state_est_callback(self):
        """Prediction step of EKF for target state; runs on timer"""
        if self.target_pose is None:
            return
        self.target_state = self.estimator.predict(self.target_state, self.predict_step_timer)

        # publish to a topic to visualize
        pred_tgt_pose_msg = PoseStamped()
        pred_tgt_pose_msg.header.frame_id = 'map'
        pred_tgt_pose_msg.header.stamp = self.get_clock().now().to_msg()
        pred_tgt_pose_msg.pose.position.x = self.target_state[0]
        pred_tgt_pose_msg.pose.position.y = self.target_state[1]
        self.predicted_tgt_pub.publish(pred_tgt_pose_msg)

    def ref_vel_callback(self, msg):
        self.ref_vel = np.array((msg.linear.x, msg.angular.z))

    def compute_target_velocity(self, prev_target_pose, target_pose, observed_pose_ts):
        target_pose_ts_nsec = observed_pose_ts.sec*1e9 + observed_pose_ts.nanosec
        prev_target_pose_ts_nsec = self.prev_target_pose_ts.sec*1e9 + self.prev_target_pose_ts.nanosec
        print("Pose ts: ", target_pose_ts_nsec, "Prev pose ts: ", prev_target_pose_ts_nsec, 
            "Diff: ", (target_pose_ts_nsec - prev_target_pose_ts_nsec)/1e9, "s",
            "Pose difference: ", target_pose[:-1] - prev_target_pose[:-1])
        if np.abs(target_pose_ts_nsec - prev_target_pose_ts_nsec) < 1000:
            print("Velocity not updated; interval too short")
            return self.target_velocity
        # for constant velocity motion model, discard the theta component of target pose
        return (target_pose[:-1] - prev_target_pose[:-1]) / ((target_pose_ts_nsec - prev_target_pose_ts_nsec)/1e9)

    def get_fov_from_lidar(self, scan, pose, stride=1):
        """Constructs FoV polygon from lidar scan
        Args:
            scan: List containing LiDAR scan ranges from /scan topic of length 1080
            pose: Array of shape (3,) representing the agent's pose as (x,y,theta)
            stride: int; downsample factor 1/stride for LiDAR scan
        Returns: Array of shape (n_vertices + 1, 2) representing agent's FoV closed polygon vertices
        """
        # get world frame coordinates of each lidar ray
        ix = np.arange(0, len(scan), stride)
        # get array of angles corresponding to the indices of the lidar scan we want to use
        angles = angle_min + angle_inc * ix
        # get the actual ranges
        ranges = scan[ix]
        # get direction vector from agent to end of lidar ray; rcosx, rsinx
        directions = np.stack([ranges * np.cos(angles), ranges * np.sin(angles)])
        # construct rotation matrix for yaw
        theta = pose[2]
        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)]
        ])

        # get world frame coordinates of the point; x_w = R@x_b + p
        fov = R @ directions + pose[:2][:,None] # (2,n_vertices)
        fov = fov.T # (n_vertices, 2)
        # sdf function needs a closed polygon so append first coordinate to the end
        fov = np.append(fov, fov[0][None, :], axis=0)

        return fov


    def control_loop(self, pose, target_pose, target_velocity, ref_vel, scan, map_info=None):
        """Main controller loop. If _map passed in is None, then raytracing
        is disabled and visibility CBF uses sampled CBF method"""

        # controller needs all this data in order to plan a single step
        if scan is not None and pose is not None and ref_vel is not None:
            print("Target pose: ", target_pose, "Target velocity: ", target_velocity, "Agent pose: ", pose, "Planner reference control: ", ref_vel)
            # process scan data and replace infs with max FoV radius, and nans too
            scan[scan == np.inf] = radius
            # u = self.solver.solve(self.pose, self.target_pose, np.zeros([0, 0, 0]), self.ref_vel)
            fov = self.get_fov_from_lidar(scan, pose, stride=2)
            # collect samples from lidar for obstacle avoidance CBF
            samples = self.solver.get_drc_samples(self.scan, n_samples=1)
            # collect samples for visibility cbf; uses raytracing and finite diff for gradient
            if self.use_sampled_cbf_visibility:
                visibility_samples = self.solver.get_visibility_cbf_samples(fov, pose, target_pose, target_velocity, map_info)
            else:  # use full cbf constraint (not sample-based)
                visibility_samples = (None, None, None)

            # solve QP with obstacle avoidance and visibility CBF
            # self.get_logger().info('Starting controller solve...')
            u = self.solver.solve_drc(self.pose, self.target_pose, self.target_velocity, 
                self.ref_vel, *samples, fov, map_info, self.use_sampled_cbf_visibility, *visibility_samples)
            
            control_twist = Twist()
            # create twist message format to publish to robot
            control_twist.linear.x = u[0]
            control_twist.angular.z = u[1]
            self.cmd_vel_pub.publish(control_twist)
            self.get_logger().info('Control command published.')
            # unlock the scan callback
            self.solver_ongoing = False
        else:
            self.get_logger().info("Some data not available. Cannot find new optimal control")
            self.solver_ongoing = False


def main(args=None):
    rclpy.init(args=args)
    node = MyControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
