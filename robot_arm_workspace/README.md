# Robot Arm Workspace

This workspace contains 4-DOF robot arm packages for simulation and control.

## Packages
- `robotArm_4DOF`: Main robot arm package
- `robot_arm_4dof`: Alternative robot arm implementation

## Build and Run
```bash
cd robot_arm_workspace
colcon build
source install/setup.bash
```

## Usage
```bash
# Launch robot arm
ros2 launch robotArm_4DOF robot_arm.launch.py

# Control joints
ros2 topic pub /joint_commands std_msgs/Float64MultiArray ...
```
