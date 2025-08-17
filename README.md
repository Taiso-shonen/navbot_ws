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

## Description
- launch file: my_navbot_gazebo.launch.py, my_navbot_gazebo.launch.xml
- description: spawns the robot in gazebo and views it in rviz. All the sensors and plugins are added.

- launch file: slam.launch.py
- description: launches my_navbot_gazebo.launch.py together with slam_toolbox. Can be used with teleop twist keyboard to move the robot. A map saver service is available with a Trigger interface.
- arguments: use_sim_time(default is True), map_name

- launch file: auto_slam.launch.py
- description: used to map an unknown environment autonomously. After reaching a certain percentage of mapping it auto-saves the map.
- arguments: use_sim_time(default is True), map_name, params_file(for navigation, can modify the default one located in the config) 

- launch file: nav.launch.py
- description: navigates to predefined checkpoints provided as waypoints.
- arguments: map, set_initial_pose(bool default to true), waypoints(yaml file containing waypoints), params_file(for navigation, can modify the default one located in the config)

