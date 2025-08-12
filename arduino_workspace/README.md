# Arduino Workspace

This workspace contains Arduino-related ROS2 packages for robot control and simulation.

## Packages
- `arduinobot_description`: Robot description files (URDF, meshes)
- `arduinobot_py_examples`: Python examples for Arduino robot control
- `arduinorobot_cpp_ex`: C++ examples for Arduino robot control

## Build and Run
```bash
cd arduino_workspace
colcon build
source install/setup.bash
```

## Usage
```bash
# Launch robot description
ros2 launch arduinobot_description display.launch.py

# Run Python examples
ros2 run arduinobot_py_examples simple_publisher
```
