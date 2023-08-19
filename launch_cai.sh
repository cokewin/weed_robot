source ~/ros_canopen/install/setup.bash
colcon build
source install/setup.bash
ros2 launch canopen_test cia402_system.launch.py