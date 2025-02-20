
# ROSNavCar: Autonomous Car Simulation with Obstacle Avoidance  

This repository contains a simulation package for an autonomous car-like robot using **ROS 2 Jazzy** and **Gazebo Harmonic**. The robot navigates toward a user-defined goal while avoiding obstacles along its path.  

## Overview  

**ROSNavCar** simulates a differential drive (car-like) robot equipped with sensors (e.g., **LiDAR**) to detect obstacles. The navigation code uses these sensor readings to perform obstacle avoidance and safely guide the robot to the target destination.  

## Features  

- **Goal Setting:**  
  Set your target destination directly in the navigation code. For example, to set the goal to `x = 5.0` and `y = 8.0`, modify:  
  ```python
  self.goal = [5.0, 8.0]
  ```
- **Obstacle Avoidance:**  
  The robot automatically detects obstacles and adjusts its path to avoid collisions.  
- **Simulation Environment:**  
  Uses **Gazebo Harmonic** for realistic simulation and **ROS 2 Jazzy** for robust communication.  
- **Visualization:**  
  RViz support is available to visualize sensor data and the planned path.  

## Requirements  

- **Operating System:**  
  **Ubuntu 24.04 (Recommended)**  
- **ROS Distribution:**  
  **ROS 2 Jazzy**  
- **Simulator:**  
  **Gazebo Harmonic**  

## Install Required ROS 2 Packages  

Make sure to install the following **ROS 2 Jazzy** packages:  

```bash
sudo apt-get update

sudo apt install -y                         \
   ros-jazzy-ros-gz                        \
   ros-jazzy-ros-gz-bridge                 \
   ros-jazzy-joint-state-publisher         \
   ros-jazzy-xacro                         \
   ros-jazzy-teleop-twist-keyboard         \
   ros-jazzy-teleop-twist-joy
```

## Installation  

1. **Clone the Repository:**  

   ```bash
   git clone https://github.com/darshmenon/rosnav.git
   cd rosnavcar
   ```

2. **Set Up Your ROS 2 Workspace:**  

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/darshmenon/rosnavcar.git
   cd ~/ros2_ws
   colcon build --symlink-install
   ```

3. **Source Your Workspace:**  

   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Setting the Navigation Goal  

The navigation goal is defined in the navigation node. You can set your desired target by editing the goal variable. For example:  

```python
self.goal = [5.0, 8.0]
```

This sets the target position at `x = 5.0` and `y = 8.0`.  

## Launching the Simulation  

After building your workspace and sourcing the environment, launch the simulation using:  

```bash
ros2 launch diff_drive_robot robot.launch.py
```

This command starts **Gazebo Harmonic** with the simulated robot and launches the navigation node, which directs the robot to the set goal while avoiding obstacles.  

## Contributing  

Contributions, issues, and feature requests are welcome! Please open an issue or submit a pull request on GitHub.  
