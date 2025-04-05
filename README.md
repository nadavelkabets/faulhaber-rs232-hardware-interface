# faulhaber-rs232-hardware-interface
A ROS2 control hardware interface for faulhaber rs232 motor controllers

## build
```bash
sudo apt install -y ros-jazzy-test-msgs ros-dev-tools ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-hardware-interface build-essential libserial-dev ros-jazzy-xacro
source /opt/ros/jazzy/setup.bash
colcon build --event-handlers console_cohesion+ --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```
