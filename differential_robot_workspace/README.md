# Differential Robot Workspace

This workspace contains packages for differential drive robot simulation and control.

## Packages
- `diffrobot_description`: Robot description and URDF files
- `diffrobot_gazebo`: Gazebo simulation configurations

## Build and Run
```bash
cd differential_robot_workspace
colcon build
source install/setup.bash
```

## Usage
```bash
# Launch in Gazebo
ros2 launch diffrobot_gazebo diffrobot_world.launch.py

# Control the robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
