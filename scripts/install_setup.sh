#!/bin/bash
set -e  # exit on first error
trap 'echo "Error occurred at line $LINENO"; exit 1' ERR

if [[ "$(basename "$PWD")" == "scripts" ]]; then
    cd ..
fi
echo "Starting UNL-UAV project setup..."

echo "Installing ROS2 Humble..."

sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

sudo rm -f /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros-latest.list
sudo apt remove -y ros2-apt-source || true
sudo sed -i '/ros2\/ubuntu/d' /etc/apt/sources.list

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-dev-tools
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

grep -qxF 'export PATH="$HOME/.local/bin:$PATH"' ~/.bashrc || echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
export PATH="$HOME/.local/bin:$PATH"

echo "ROS2 Humble installation complete."

echo "Installing PX4 Autopilot..."
cd ..
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot
make px4_sitl
echo "PX4 Autopilot installation complete."

echo "Installing Micro-XRCE-DDS Agent..."
cd ..
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
echo "Micro-XRCE-DDS Agent installation complete."

echo "UNL-UAV project setup complete!"