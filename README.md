# UNL-UAV Autonomous Drone Project

This project is the work of the UAV subdivision of the Aerospace Club at the University of Nebraska-Lincoln. The objective of the project is to create a fully autonomous quadcopter to deliver two payloads at two different locations before returning home. 

## Setup

To set up the UNL-UAV project, including ROS2 Humble, PX4 Autopilot, and Micro-XRCE-DDS Agent, follow these steps:

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/mmandernach13/UNL-UAV.git --recursive
    ```

2.  **Navigate into the repository:**
    ```bash
    cd UNL-UAV
    ```

3.  **Run the setup script:**
    ```bash
    ./scripts/install_setup.sh
    ```
    This script will:
    *   Install ROS2 Humble and its development tools.
    *   Build your ROS2 workspace.
    *   Install PX4 Autopilot and its dependencies.
    *   Install the Micro-XRCE-DDS Agent.

    The PX4 Autopilot and Micro-XRCE-DDS Agent will be installed in the parent directory of the `UNL-UAV` repository, creating the following structure:
    ```
    your_dir/
    --- PX4-Autopilot/
    --- Micro-XRCE-DDS-Agent/
    --- UNL-UAV/
    ```

## Usage

To start a basic simulation with `px4_sitl` run the following command from the `UNL-UAV` directory:
```bash
./scripts/launch_sim.sh
```
Running this over an SSH connection will start the simulation without the GUI, and running it on the offboard computer directly will start the Gazebo GUI. 

Running the rest of the ROS2 nodes is still a work in progress as they are not written yet. 
