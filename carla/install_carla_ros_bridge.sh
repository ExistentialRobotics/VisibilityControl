#!/usr/bin/env bash
set -euo pipefail

echo "=========================================="
echo "  CARLA ROS Bridge Installation"
echo "=========================================="

# Check if ROS2 Humble is installed
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "Error: ROS2 Humble not found. Please run install_ros2.sh first." >&2
  exit 1
fi

# Check if conda environment erl_vc is active
if [ -z "${CONDA_DEFAULT_ENV:-}" ] || [ "$CONDA_DEFAULT_ENV" != "erl_vc" ]; then
  echo "Error: Please activate conda environment 'erl_vc' first:" >&2
  echo "  conda activate erl_vc" >&2
  exit 1
fi

echo "[1/5] Sourcing ROS2 Humble environment..."
source /opt/ros/humble/setup.bash

echo "[2/5] Installing ROS2 system dependencies..."
sudo apt update
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-vision-opencv \
  ros-humble-pcl-conversions \
  ros-humble-ackermann-msgs \
  ros-humble-derived-object-msgs

# Install CARLA Python API (matching CARLA version 0.9.14)
CARLA_VERSION="0.9.14"
CARLA_EGG="carla-${CARLA_VERSION}-py3.7-linux-x86_64.egg"
CARLA_ROOT="$(pwd)/CARLA_${CARLA_VERSION}"

if [ -d "$CARLA_ROOT" ]; then
  echo "[3/5] Installing CARLA Python API from $CARLA_ROOT..."
  if [ -f "$CARLA_ROOT/PythonAPI/carla/dist/$CARLA_EGG" ]; then
    pip install "$CARLA_ROOT/PythonAPI/carla/dist/$CARLA_EGG"
    echo "CARLA Python API installed to conda environment"
  else
    echo "Warning: CARLA Python egg not found at $CARLA_ROOT/PythonAPI/carla/dist/$CARLA_EGG" >&2
    echo "Skipping CARLA Python API installation. Install manually if needed." >&2
  fi
else
  echo "[3/5] CARLA installation not found at $CARLA_ROOT"
  echo "Skipping CARLA Python API installation. Install after running install_carla.sh" >&2
fi

# Create workspace for carla-ros-bridge
WORKSPACE_DIR="$HOME/carla_ros_ws"

echo "[4/5] Setting up carla-ros-bridge workspace at $WORKSPACE_DIR..."
mkdir -p "$WORKSPACE_DIR/src"
cd "$WORKSPACE_DIR/src"

if [ -d "ros-bridge" ]; then
  echo "Repository already exists, pulling latest changes..."
  cd ros-bridge
  git pull
  cd ..
else
  git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git
fi

echo "[5/5] Building carla-ros-bridge with colcon..."
cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
colcon build --symlink-install

echo ""
echo "=========================================="
echo "  CARLA ROS Bridge Installation Complete!"
echo "=========================================="
echo ""
echo "Workspace: $WORKSPACE_DIR"
echo ""
echo "To use the bridge (with conda env 'erl_vc' activated):"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $WORKSPACE_DIR/install/setup.bash"
echo ""