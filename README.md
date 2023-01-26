# CRAZYSWARM APPLICATION (ROS2)

Crazyswarm application layer, which provides a **path planning layer** to reach desired setpoints, **apriltag detection** for relocalization to global map and on-user commands.

## Environment setup and compile
```bash
mkdir -p crazyswarm2_ws/src
cd ~/crazyswarm2_ws/src

# git clone these 5 repositories
git clone git@github.com:matthewoots/apriltag_msgs.git --branch crazyflie
git clone git@github.com:matthewoots/apriltag_ros.git --branch crazyflie
git clone git@github.com:matthewoots/crazyswarm2.git --branch crazyflie --recursive
git clone --branch ros2 --recursive https://github.com/IMRCLab/motion_capture_tracking.git
git clone git@github.com:matthewoots/crazyswarm2_application.git --recursive

# install other dependencies
sudo apt-get install -y \
ros-galactic-apriltag \
libboost-program-options-dev \
libusb-1.0-0-dev
pip3 install rowan

# build
cd .. # to <workspace-directory>
# build the crazyswarm environment
colcon build --symlink-install
# source the bash file
source install/setup.bash
```

## Launch
```bash
ros2 launch crazyflie launch.py rviz:=none # Real
ros2 launch crazyflie launch.py backend:=sim rviz:=none # Simulation

ros2 launch crazyswarm_application launch.py # main handler node
ros2 launch crazyswarm_application mission.py # start mission
ros2 launch crazyswarm_application rviz.py # visualization
```

## Some test commands without crazyswarm_application
```bash
# to give a single drone go to service call without using the node
ros2 service call '/cf1/go_to' \
'crazyflie_interfaces/srv/GoTo' \
'{group_mask: 0, relative: false, goal: {x: 5, y: 1, z: 1}, yaw: 0.707, duration: {sec: 2.0, nanosec: 0}}'

# to give a single drone go to velocity service call without using the node
ros2 topic pub -r 10 '/cf1/cmd_vel_legacy' \
'crazyflie_interfaces/msg/VelocityWorld' \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, vel: {x: 0.2, y: 0.2, z: 0.0}, yaw_rate: 0.0}'

# to give a takeoff all service call without using the node
ros2 service call '/all/takeoff' \
'crazyflie_interfaces/srv/Takeoff' \
'{group_mask: 0, height: 1.0, duration: {sec: 2.0, nanosec: 0}}'

# to give a land all service call without using the node
ros2 service call '/all/land' \
'crazyflie_interfaces/srv/Land' \
'{group_mask: 0, height: 0.0, duration: {sec: 2.0, nanosec: 0}}'

# give takeoff all command
ros2 topic pub /user crazyswarm_application/msg/UserCommand \
'{cmd: 'takeoff_all', uav_id: [], goal: {x: 0.0, y: 0.0, z: 0.0}, yaw: 0.0}' --once

# give land all command
ros2 topic pub /user crazyswarm_application/msg/UserCommand \
'{cmd: 'land_all', uav_id: [], goal: {x: 0.0, y: 0.0, z: 0.0}, yaw: 0.0}' --once

# give go_to command
ros2 topic pub /user crazyswarm_application/msg/UserCommand \
'{cmd: 'goto', uav_id: ['cf1'], goal: {x: 5.0, y: 0.0, z: 1.0}, yaw: 0.707}' --once

# give go_to command
ros2 topic pub /user crazyswarm_application/msg/UserCommand \
'{cmd: 'goto', uav_id: ['cf1', 'cf2', 'cf3'], goal: {x: 5.0, y: 0.0, z: 1.0}, yaw: 0.707}' --once
```