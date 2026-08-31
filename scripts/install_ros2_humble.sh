#!/usr/bin/env bash
# One-time ROS 2 Humble install for Ubuntu 22.04 (works inside WSL2).
# Run with: bash scripts/install_ros2_humble.sh
set -euo pipefail

sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-rviz2 \
  python3-colcon-common-extensions \
  python3-pytest

grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc \
  || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

echo
echo "Done. Open a new shell (or 'source ~/.bashrc'), then build the workspace:"
echo "  cd ~/aeb_ros2_demo && colcon build && source install/setup.bash"
