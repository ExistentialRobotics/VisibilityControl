#!/bin/bash

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash
source carla-ros-bridge/install/setup.bash

BUILD_DIR="cbf_ros/target_publisher/build"
INSTALL_DIR="cbf_ros/target_publisher/install"

if [ ! -d "$BUILD_DIR" ] || [ ! -d "$INSTALL_DIR" ]; then
  cd cbf_ros/target_publisher
  colcon build
  cd ../..
fi

source cbf_ros/target_publisher/install/setup.bash
source ~/anaconda3/etc/profile.d/conda.sh
conda activate erl_vc
mkdir -p logs
export PYTHONPATH=$CONDA_PREFIX/lib/python3.10/site-packages:$PYTHONPATH
export PATH=$CONDA_PREFIX/bin:$PATH

# Array to hold process IDs of background jobs
PIDS=()

# Function to check if a process is running
check_process() {
    if ps -p $1 > /dev/null
    then
        echo "$2 is running with PID $1"
    else
        echo "$2 failed to start"
        exit 1
    fi
}

# Function to clean up background jobs
cleanup() {
    echo "Terminating all background processes gracefully..."
    for pid in "${PIDS[@]}"; do
        echo "Terminating PID $pid"
	kill $pid
    done
    wait
    kill $CARLA_SIMULATOR_PID
    echo "All processes have been terminated."
    exit 0
}

# Trap SIGINT to ensure cleanup is called on Ctrl+C
trap cleanup SIGINT

# Run CARLA Simulator
echo "Starting CARLA Simulator..."
CARLA_0.9.14/CarlaUE4.sh > logs/carla_simulator.log 2>&1 &
CARLA_SIMULATOR_PID=$!
# PIDS+=($CARLA_SIMULATOR_PID)
sleep 5
check_process $CARLA_SIMULATOR_PID "CARLA Simulator"

# Run CARLA ROS Bridge
echo "Starting CARLA ROS Bridge..."
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py town:=Env02 fixed_delta_seconds:=0.0 > logs/carla_ros_bridge.log 2>&1 &
CARLA_ROS_BRIDGE_PID=$!
PIDS+=($CARLA_ROS_BRIDGE_PID)
sleep 5
check_process $CARLA_ROS_BRIDGE_PID "CARLA ROS Bridge"

# Run Static Transform Publisher
echo "Starting Static Transform Publisher..."
ros2 run tf2_ros static_transform_publisher 2 0 2 0 0 0 ego_vehicle camera_link > logs/static_transform.log 2>&1 &
STATIC_TRANSFORM_PID=$!
PIDS+=($STATIC_TRANSFORM_PID)
sleep 2
check_process $STATIC_TRANSFORM_PID "Static Transform Publisher"

# Run Depthimage to Laserscan
echo "Starting Depthimage to Laserscan..."
ros2 launch depthimage_to_laserscan depthimage_to_laserscan-launch.py > logs/depthimage_to_laserscan.log 2>&1 &
DEPTHIMAGE_TO_LASERSCAN_PID=$!
PIDS+=($DEPTHIMAGE_TO_LASERSCAN_PID)
sleep 2
check_process $DEPTHIMAGE_TO_LASERSCAN_PID "Depthimage to Laserscan"

# Run SLAM Toolbox
echo "Starting SLAM Toolbox..."
ros2 launch slam_toolbox online_sync_launch.py slam_params_file:=carla/configs/mapper_params_online_sync.yaml > logs/slam_toolbox.log 2>&1 &
SLAM_TOOLBOX_PID=$!
PIDS+=($SLAM_TOOLBOX_PID)
sleep 2
check_process $SLAM_TOOLBOX_PID "SLAM Toolbox"

# Run Target Publisher
echo "Starting Target Publisher..."
python3 cbf_ros/target_pub.py > logs/target_publisher.log 2>&1 &
TARGET_PUBLISHER=$!
PIDS+=($TARGET_PUBLISHER)
sleep 2
check_process $TARGET_PUBLISHER "Target Publisher"

# Run RViz
echo "Starting RViz..."
rviz2 -d configs/carla.rviz > logs/rviz2.log 2>&1 &
RVIZ2=$!
PIDS+=($RVIZ2)
sleep 5
check_process $RVIZ2 "RViz2"

echo "Starting CBF Controller..."
cd cbf_ros
conda activate erl_vc
python3 init_full_loop.py > ../logs/CBF.log 2>&1 &
CBF=$!
PIDS+=($CBF)
sleep 5
check_process $CBF "CBF"

# Function to display logs
tail_logs() {
    tail -f \
    ../logs/carla_ros_bridge.log \
    ../logs/static_transform.log \
    ../logs/depthimage_to_laserscan.log \
    ../logs/slam_toolbox.log \
    ../logs/carla_simulator.log \
    ../logs/rviz2.log \
    ../logs/CBF.log
}

# Monitor the logs
tail_logs

# Wait for all background jobs to finish
wait
