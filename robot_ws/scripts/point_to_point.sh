cd ~/robot_ws/
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_move
export ROS_DOMAIN_ID=44
source ~/robot_ws/install/setup.bash
ros2 run robot_move send_point
