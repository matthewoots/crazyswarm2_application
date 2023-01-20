cmake_minimum_required(VERSION 3.8)
project(crazyswarm_application)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(crazyflie_interfaces REQUIRED)
find_package(motion_capture_tracking_interfaces REQUIRED)
find_package(apriltag_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/UserCommand.msg"
  "msg/AgentState.msg"
  "msg/AgentsStateFeedback.msg"
  )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  include
)

add_executable(${PROJECT_NAME}_node
  src/crazyswarm_app.cpp
  src/april_tag.cpp
  src/handling_planning.cpp
)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME})
rosidl_target_interfaces(${PROJECT_NAME}_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  std_srvs
  crazyflie_interfaces
  motion_capture_tracking_interfaces
  apriltag_msgs
)

add_executable(mission_node
  src/mission_node.cpp
)

add_dependencies(mission_node ${PROJECT_NAME})
rosidl_target_interfaces(mission_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(mission_node
  rclcpp
  sensor_msgs
  std_srvs
  crazyflie_interfaces
)

# Install C++ executables
install(TARGETS
  ${PROJECT_NAME}_node
  mission_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
