#!/usr/bin/env bash
set -euo pipefail

echo "=========================================="
echo "  CARLA Simulation Environment Setup"
echo "  Complete Installation Script"
echo "=========================================="
echo ""

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check if conda environment exists
if ! conda env list | grep -q "erl_vc"; then
  echo "Error: Conda environment 'erl_vc' not found." >&2
  echo "Please create the environment first:" >&2
  echo "  conda create -n erl_vc python=3.9" >&2
  echo "  conda activate erl_vc" >&2
  echo "  pip install -r requirements.txt" >&2
  exit 1
fi

# Check if conda environment is active
if [ -z "${CONDA_DEFAULT_ENV:-}" ] || [ "$CONDA_DEFAULT_ENV" != "erl_vc" ]; then
  echo "Error: Please activate conda environment 'erl_vc' first:" >&2
  echo "  conda activate erl_vc" >&2
  exit 1
fi

echo "Conda environment 'erl_vc' is active. Proceeding with installation..."
echo ""

# Step 1: Install CARLA
echo "=========================================="
echo "  Step 1/3: Installing CARLA Simulator"
echo "=========================================="
echo ""

if [ -f "$SCRIPT_DIR/install_carla.sh" ]; then
  cd "$SCRIPT_DIR"
  chmod +x install_carla.sh
  ./install_carla.sh
  if [ $? -ne 0 ]; then
    echo "Error: CARLA installation failed." >&2
    exit 1
  fi
else
  echo "Error: install_carla.sh not found in $SCRIPT_DIR" >&2
  exit 1
fi

echo ""
echo "CARLA installation completed successfully!"
echo ""
sleep 2

# Step 2: Install ROS2 Humble
echo "=========================================="
echo "  Step 2/3: Installing ROS2 Humble"
echo "=========================================="
echo ""

if [ -f "$SCRIPT_DIR/install_ros2.sh" ]; then
  cd "$SCRIPT_DIR"
  chmod +x install_ros2.sh
  ./install_ros2.sh
  if [ $? -ne 0 ]; then
    echo "Error: ROS2 installation failed." >&2
    exit 1
  fi
else
  echo "Error: install_ros2.sh not found in $SCRIPT_DIR" >&2
  exit 1
fi

echo ""
echo "ROS2 installation completed successfully!"
echo ""
sleep 2

# Step 3: Install CARLA ROS Bridge
echo "=========================================="
echo "  Step 3/3: Installing CARLA ROS Bridge"
echo "=========================================="
echo ""

if [ -f "$SCRIPT_DIR/install_carla_ros_bridge.sh" ]; then
  cd "$SCRIPT_DIR"
  chmod +x install_carla_ros_bridge.sh
  ./install_carla_ros_bridge.sh
  if [ $? -ne 0 ]; then
    echo "Error: CARLA ROS Bridge installation failed." >&2
    exit 1
  fi
else
  echo "Error: install_carla_ros_bridge.sh not found in $SCRIPT_DIR" >&2
  exit 1
fi

echo ""
echo "=========================================="
echo "  ✓ All Installations Complete!"
echo "=========================================="
echo ""
echo "Summary:"
echo "  - CARLA Simulator: $SCRIPT_DIR/CARLA_0.9.14"
echo "  - ROS2 Humble: /opt/ros/humble"
echo "  - CARLA ROS Bridge: ~/carla_ros_ws"
echo ""
echo "Next Steps:"
echo "  1. Source ROS2 setup:"
echo "     source /opt/ros/humble/setup.bash"
echo ""
echo "  2. Source CARLA ROS Bridge workspace:"
echo "     source ~/carla_ros_ws/install/setup.bash"
echo ""
echo "  3. Launch CARLA simulator:"
echo "     cd $SCRIPT_DIR/CARLA_0.9.14 && ./CarlaUE4.sh"
echo ""
echo "  4. In another terminal (with conda env and ROS2 sourced):"
echo "     ros2 launch carla_ros_bridge carla_ros_bridge.launch.py"
echo ""
echo "Consider adding to ~/.bashrc:"
echo "  echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc"
echo "  echo 'source ~/carla_ros_ws/install/setup.bash' >> ~/.bashrc"
echo ""