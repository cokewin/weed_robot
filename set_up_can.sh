source ~/ros_canopen/install/setup.bash
colcon build
source install/setup.bash
ros2 launch canopen_tests robot_control_setup.launch.py