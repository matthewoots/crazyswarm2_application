# CRAZYSWARM APPLICATION (ROS2)

Crazyswarm application layer, which provides a **path planning layer** to reach desired setpoints, **apriltag detection** for relocalization to global map and static and reciprocal avoidance and on-user commands. Also included capability, to "allocate" themselves to targets as seen below.

## Simulation Example
![sample](media/sample.gif)

## Real Life Relocalization Example
![sample](media/relocalization.gif)

## Important Dependencies
1. [For relocalization] `gtsam` at https://github.com/borglab/gtsam using version 4.1.1
2. [For reciprocal avoidance] `kdtree` to organize agents into a kdtree
3. [For reciprocal avoidance] `orca` has been taken from `agent.c` and `agent.h`and heavily modified to work with this module (making it more of a standalone) https://github.com/snape/RVO2-3D
4. [For static avoidance] `3dvg` for visibility graph planning in structured environment (will add in soon)
5. [Crazyflie firmware for mellinger velocity control] `crazyflie-firmware` at my fork https://github.com/matthewoots/crazyflie-firmware
6. The rest of the additional modules below are forks since there are modifications I have done to make them work with this module

## Dependencies setup
1. `Crazyflie-firmware`
```bash
mkdir crazyflie && cd crazyflie
git clone git@github.com:matthewoots/crazyflie-firmware.git
cd crazyflie-firmware
make cf2_defconfig
make bindings_python
echo "export PYTHONPATH=~/crazyflie/crazyflie-firmware:$PYTHONPATH" >> ~/.bashrc
```

2. `ROS2 Galactic`

Follow installation insturctions at https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
```bash
# Add this line to bashrc to always source ros2 galactic (optional)
echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
```

3. `GTSAM`
```bash
#install gtsam
cd ~/crazyflie
git clone https://github.com/borglab/gtsam
cd gtsam && git checkout 4.1.1
mkdir build
cd build
cmake ..
make
sudo make install
```

4. `Other dependcies`
```bash
# install other dependencies
sudo apt-get install -y \
ros-galactic-apriltag \
libboost-program-options-dev \
libusb-1.0-0-dev
pip3 install rowan

#very important for gtsam
echo $LD_LIBRARY_PATH

#gtsam library is found inside here
export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH
```

## Environment setup and compile
```bash
cd ~/crazyflie
mkdir -p crazyswarm2_ws/src
cd ~/crazyswarm2_ws/src


# git clone these 6 repositories
git clone git@github.com:matthewoots/apriltag_msgs.git --branch crazyflie
git clone git@github.com:matthewoots/apriltag_ros.git --branch crazyflie
git clone git@github.com:matthewoots/crazyswarm2.git --branch crazyflie --recursive
git clone --branch ros2 --recursive https://github.com/IMRCLab/motion_capture_tracking.git
git clone git@github.com:matthewoots/crazyswarm2_application.git --recursive
git clone git@github.com:teamspatzenhirn/rviz_2d_overlay_plugins.git


# build
# Deactivate conda env if applicable before colcon build
cd .. # to <workspace-directory>

# build the crazyswarm environment
colcon build --symlink-install

# source the bash file,
source install/setup.bash
```

For mission files, there are various missions sample files that are in `launch/mission/*`, they are in yaml format and more information can be seen in `launch/config.yaml`

Launching `ros2 launch crazyswarm_application mission.py` will load the files after you have compiled and you can choose it as an input
```
user@user:~/crazyswarm2_ws$ ros2 launch crazyswarm_application mission.py
[INFO] [launch]: Default logging verbosity is set to INFO
0 .  1_agent_high.yaml
1 .  takeoff_land.yaml
2 .  1_agent_move.yaml
3 .  sample.yaml
4 .  empty.yaml
5 .  sample1.yaml
6 .  3_agent_orca.yaml
7 .  3_agent_orca_eliminate.yaml
What is your mission file?
1
Mission chosen is takeoff_land.yaml
```


## Launch
```bash
ros2 launch crazyflie launch.py rviz:=none # Real
ros2 launch crazyflie launch.py backend:=sim rviz:=none # Simulation

ros2 launch crazyswarm_application launch.py sim:=true # main handler node for simulation
ros2 launch crazyswarm_application launch.py # main handler node for real
ros2 launch crazyswarm_application mission.py # start mission
ros2 launch crazyswarm_application rviz.py # visualization
```

# [Archive]
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
