import rosbag2_py
import numpy as np

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose


def read_ros2_bag(bag_file):
    # Initialize reader
    storage_options = rosbag2_py.StorageOptions(uri=bag_file, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Prepare data containers
    scan_data = []
    pose_data = []

    while reader.has_next():
        (topic, data, t) = reader.read_next()

        if topic == '/scan':
            scan = deserialize_message(data, LaserScan)
            scan_data.append(np.array(scan.ranges))

        if topic == '/pose':
            pose = deserialize_message(data, Pose)
            pose_data.append(np.array([pose.position.x, pose.position.y, pose.position.z,
                                       pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]))

    return np.array(scan_data), np.array(pose_data)


if __name__ == '__main__':
    bag_file = 'rosbag2_2024_08_01-18_06_07_0.db3'
    scan_array, pose_array = read_ros2_bag(bag_file)

    # Save to .npy files if needed
    np.save('scan_data.npy', scan_array)
    np.save('pose_data.npy', pose_array)

    print('Scan data shape:', scan_array.shape)
    print('Pose data shape:', pose_array.shape)
