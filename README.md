```bash
colcon build --symlink-install

source install/setup.bash
ros2 launch crazyflie launch.py backend:=sim
ros2 launch crazyswarm_application launch.py

# to give a single drone go to service call without using the node
ros2 service call '/cf1/go_to' \
'crazyflie_interfaces/srv/GoTo' \
'{group_mask: 0, relative: false, goal: {x: 1, y: 1, z: 1}, yaw: 90, duration: {sec: 5.0, nanosec: 0}}'

# to give a takeoff all service call without using the node
ros2 service call '/all/takeoff' \
'crazyflie_interfaces/srv/Takeoff' \
'{group_mask: 0, height: 1.0, duration: {sec: 4.0, nanosec: 0}}'

# to give a land all service call without using the node
ros2 service call '/all/land' \
'crazyflie_interfaces/srv/Land' \
'{group_mask: 0, height: 0.0, duration: {sec: 2.0, nanosec: 0}}'

# give takeoff command
ros2 topic pub /common crazyswarm_application/msg/UserCommand \
'{cmd: 'takeoff', uav_id: [], goal: {x: 0.0, y: 0.0, z: 0.0}, yaw: 0.0}' --once
```