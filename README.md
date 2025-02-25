# ROS 2 Navigation and SLAM with Nav2 and Gazebo Harmonic

## Overview
This project utilizes **Nav2** and **SLAM Toolbox** for robot navigation and obstacle avoidance in a simulated environment. The robot autonomously detects obstacles and adjusts its path to avoid collisions while leveraging **Gazebo Harmonic** for realistic simulations and **ROS 2 Jazzy** for communication and control.

## Features
- **Obstacle Avoidance**: The robot automatically detects obstacles and adjusts its trajectory to avoid collisions.
- **Simulation Environment**: Uses **Gazebo Harmonic** for high-fidelity simulations.
- **Visualization**: RViz support for real-time sensor data and path visualization.

## Requirements
### Operating System:
- **Ubuntu 24.04** (Recommended)

### ROS Distribution:
- **ROS 2 Jazzy**

### Simulator:
- **Gazebo Harmonic**

## Installation
### Install Required ROS 2 Packages
Make sure to install the following ROS 2 Jazzy packages:
```bash
sudo apt-get update
sudo apt install -y \   
   ros-jazzy-ros-gz \  
   ros-jazzy-ros-gz-bridge \  
   ros-jazzy-joint-state-publisher \  
   ros-jazzy-xacro \  
   ros-jazzy-teleop-twist-keyboard \  
   ros-jazzy-teleop-twist-joy \  
   ros-jazzy-nav2-bringup \  
   ros-jazzy-slam-toolbox
```

### Create a ROS 2 Workspace (if you don’t already have one)
```bash
mkdir -p ~/roscar/src
cd ~/roscar
colcon build --symlink-install
```

### Clone This Repository into the Workspace
```bash
cd ~/roscar/src
git clone https://github.com/darshmenon/rosnav.git
```
(The repository will be cloned into a folder named `rosnav`.)

### Build the Workspace
```bash
cd ~/roscar
colcon build --symlink-install
```

### Source Your Workspace
```bash
source ~/roscar/install/setup.bash
```

## Running Navigation and SLAM
### Start Navigation with Nav2
To launch the navigation stack:
```bash
ros2 launch diff_drive_robot nav2.launch.py
```
Once Nav2 is running, set the navigation goal pose in **RViz**.

### Run SLAM and Generate a Map
To start SLAM and generate a map:
```bash
ros2 launch diff_drive_robot slam.launch.py
```
Alternatively, you can modify `robot.launch.py` to comment out the navigation node and manually set navigation goals.

## Manual Robot Control
If you prefer manual control, use the keyboard teleoperation tool:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
This allows you to control the robot using the arrow keys.

## Goal Setting(alternatively)
Set your target destination directly in the navigation script `navigation.py`. For example, to set the goal at `x = 5.0` and `y = 8.0`, modify the script:
```python
self.nav_to_pose(x=5.0, y=8.0)
```

## Contributing
Feel free to contribute to this project by submitting issues or pull requests.



