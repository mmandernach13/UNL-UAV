# UNL-UAV Autonomous Drone Project

This project is the work of the UAV subdivision of the Aerospace Club at the University of Nebraska-Lincoln. The objective of the project is to create a fully autonomous quadcopter to deliver two payloads at two different locations before returning home. 

## Prerequisites

Running the ROS2 nodes requires having ROS2 installed which can be done [here](https://docs.ros.org/en/humble/Installation.html). The version of ROS2 used is ROS2 Humble which requires the Ubuntu-22.04 version of Linux. It is reccomended to use a Jetson Nano or other single board computer to act as the UAV's offboard computer. Although you can run the simulations on a desktop running Ubuntu-22.04 as long as the dependancies are installed. The Jetson Nano Orin was used for this project. 

You will need to have the PX4 autopilot software and Micro-XRCE-DDS Agent installed. In order for the ``launch_sim.sh`` and ``kill_sim.sh`` to work properly you will need to have the directories installed in the following configuration:
```
your_dir/
--- PX4/
--- Micro-XRCE-DDS/
--- this_repo/
```
To install and build the PX4 autopilot follow the instructions [here](https://docs.px4.io/main/en/dev_setup/building_px4). 

To install and run the Micro-XRCE-DDS Agent follow the instructions [here](https://docs.px4.io/main/en/middleware/uxrce_dds)

## Usage

Start by cloning the repo with 
```bash
git clone https://github.com/mmandernach13/UNL-UAV.git --recursive
```

To start a basic simulation with `px4_sitl` run the following command:
```bash
cd UNL-UAV && ./scripts/launch_sim.sh
```
Running this over an SSH connection will start the simulation without the GUI, and running it on the offboard computer directly will start the Gazebo GUI. 

Running the rest of the ROS2 nodes is still a work in progress as they are not written yet. 
