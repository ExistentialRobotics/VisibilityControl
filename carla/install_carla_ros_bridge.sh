#!/usr/bin/env bash
# install_carla_ros_bridge_ros2.sh
# Follows: https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/
pip install empy==3.3.4
pip install catkin_pkg
pip install lark
ROS_DISTRO="humble"
WS=carla-ros-bridge
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

# Optionally pass the absolute path to your CARLA *.egg as $1
CARLA_EGG="${1:-}"

# Source ROS 2
source "${ROS_SETUP}"

# Create workspace
mkdir -p "${WS}/src"
cd "${WS}"

# Clone ros-bridge with submodules (per docs)
if [[ ! -d src/ros-bridge ]]; then
  git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git src/ros-bridge
fi

# Resolve dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sed -i 's|#include <tf2_eigen/tf2_eigen.h>|#include <tf2_eigen/tf2_eigen/tf2_eigen.hpp>|' src/ros-bridge/pcl_recorder/include/PclRecorderROS2.h
sed -i 's|0.9.13|0.9.14|' src/ros-bridge/carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION
# Build
colcon build

# Setup environment
source install/setup.bash
sed -i 's|numpy.bool|numpy.bool_|' install/carla_ros_bridge/lib/python3.10/site-packages/carla_ros_bridge/camera.py
sed -i 's|self.get_topic_prefix()|self.get_topic_prefix()+"_pcl2"|' install/carla_ros_bridge/lib/python3.10/site-packages/carla_ros_bridge/lidar.py
sed -i 's|self.get_topic_prefix()|self.get_topic_prefix()+"_collision"|' install/carla_ros_bridge/lib/python3.10/site-packages/carla_ros_bridge/collision_sensor.py
sed -i 's|self.get_topic_prefix()|self.get_topic_prefix()+"_imu"|' install/carla_ros_bridge/lib/python3.10/site-packages/carla_ros_bridge/imu.py

# Run
# ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py town:=Env02 fixed_delta_seconds:=0.0
