#!/usr/bin/env bash

echo "=========================================="
echo "  ROS2 Humble Installation for Ubuntu 22.04"
echo "=========================================="

# Check if running on Ubuntu 22.04
if [ -f /etc/os-release ]; then
  . /etc/os-release
  if [ "$VERSION_ID" != "22.04" ]; then
    echo "Warning: This script is designed for Ubuntu 22.04. Current version: $VERSION_ID" >&2
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
      exit 1
    fi
  fi
else
  echo "Warning: Cannot verify Ubuntu version" >&2
fi

echo "[1/6] Setting locale..."
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "[2/6] Setting up sources..."
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y

echo "[3/6] Adding ROS2 GPG key..."
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "[4/6] Adding ROS2 repository..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "[5/6] Updating package index and installing ROS2 Humble Desktop..."
sudo apt update
sudo apt install -y ros-humble-desktop

echo "[6/6] Installing ROS2 development tools..."
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-dev-tools

# Initialize rosdep if not already done
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  echo "Initializing rosdep..."
  sudo rosdep init
fi
rosdep update

echo ""
echo "=========================================="
echo "  ROS2 Humble Installation Complete!"
echo "=========================================="
echo ""
echo "To use ROS2, source the setup file:"
echo "  source /opt/ros/humble/setup.bash"
echo ""
echo "Add to your ~/.bashrc if needed:"
echo "  echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc"
echo ""