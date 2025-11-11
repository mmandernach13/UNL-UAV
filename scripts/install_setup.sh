#!/bin/bash

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
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-dev-tools
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
cd ros2_ws
colcon build
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
echo "ROS2 Humble and workspace installation complete."

echo "Installing PX4 Autopilot..."
cd ..
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot
make px4_sitl_default gazebo
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