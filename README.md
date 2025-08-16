# NavBot

A ROS 2 mobile robot project for autonomous navigation and mapping in simulation.  
This project was created as part of my learning journey with ROS 2, SLAM, and Nav2.

## Features
- Custom URDF robot model (with sensors, plugins, and xacro support)
- Simulation in Gazebo
- Visualization in RViz
- Teleoperation using keyboard
- SLAM (SLAM Toolbox)
- Autonomous navigation using Nav2

## Repository Structure
- `my_navbot_description/` → URDF, sensors and robot model files
- `my_navbot_bringup/` → launch files for SLAM, navigation, and simulation
- `my_navbot_interfaces/`  → custom srv and msg definitions
- `my_navbot_tools/`  → custom created nodes for mapping, map saving, navtopose client and other nodes under development

## Requirements
- ROS 2 Jazzy 
- Gazebo Sim 
- Navigation2
- SLAM Toolbox
- Numpy

## Usage
Clone into your workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/Taiso-shonen/navbot_ws.git
colcon build

