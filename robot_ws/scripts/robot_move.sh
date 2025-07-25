cd ~/robot_ws/
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=44
sudo chmod 777/dev/ttyUSB0
sudo chmod 777/dev/ttyUSB1
ros2 launch robot_move robot_move.launch.py
