source ~/ros2/install/setup.bash
colcon build
source install/setup.bash
ros2 launch canopen_sec_test robot_control_setup.launch.py