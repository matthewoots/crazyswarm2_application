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
find_package(rosgraph_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)

find_package(GTSAM REQUIRED)

find_package(crazyflie_interfaces REQUIRED)
find_package(apriltag_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/UserCommand.msg"
  "msg/AgentState.msg"
  "msg/NamedPose.msg"
  "msg/NamedPoseArray.msg"
  "msg/AgentsStateFeedback.msg"
  "srv/Agents.srv"
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
  src/orca
  external/kdtree
  external/CSVWriter/include
  ${GTSAM_INCLUDE_DIR}
)

add_subdirectory(external/3dvg)

set(APPLICATION_SRC
  src/crazyswarm_app.cpp
  src/handler/april_tag.cpp
  src/handler/planning.cpp)

set(ORCA_SRC
  src/orca/agent.cc)

# add application node, including april_tag and planning handler
add_executable(${PROJECT_NAME}_node
  ${APPLICATION_SRC} 
  src/common.cpp
  ${ORCA_SRC}
  external/kdtree/kdtree.c
  external/3dvg/src/visibility.cpp
  external/3dvg/VisiLibity1/src/visilibity.cpp
)
target_include_directories(
  ${PROJECT_NAME}_node PUBLIC
  external/3dvg/src
  external/3dvg/VisiLibity1/src
)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_node 
  gtsam
  "${PROJECT_NAME}__rosidl_typesupport_cpp")
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  std_srvs
  crazyflie_interfaces
  visualization_msgs
  tf2_ros
  apriltag_msgs
)

# add mission node
add_executable(mission_node src/mission_node.cpp src/common.cpp)
add_dependencies(mission_node ${PROJECT_NAME})

target_link_libraries(mission_node visibility_graph "${PROJECT_NAME}__rosidl_typesupport_cpp")
ament_target_dependencies(mission_node
  rclcpp
  sensor_msgs
  std_srvs
  crazyflie_interfaces
)

# add visualization node
add_executable(visualization_node 
  src/visualization/rviz_visualizer.cpp src/common.cpp)
add_dependencies(visualization_node ${PROJECT_NAME})
target_link_libraries(visualization_node visibility_graph "${PROJECT_NAME}__rosidl_typesupport_cpp")
ament_target_dependencies(visualization_node
  rclcpp
  sensor_msgs
  std_srvs
  visualization_msgs
  crazyflie_interfaces
  tf2_ros
  rviz_2d_overlay_msgs
)

# add april_detection_proxy node
add_executable(april_detection_proxy_node 
  src/april_detection_proxy.cpp src/common.cpp)
add_dependencies(april_detection_proxy_node ${PROJECT_NAME})
target_link_libraries(april_detection_proxy_node visibility_graph "${PROJECT_NAME}__rosidl_typesupport_cpp")
ament_target_dependencies(april_detection_proxy_node
  rclcpp
  sensor_msgs
  std_srvs
  visualization_msgs
  rosgraph_msgs
  apriltag_msgs
  crazyflie_interfaces
)

# Install C++ executables
install(TARGETS
  ${PROJECT_NAME}_node
  mission_node
  visualization_node
  april_detection_proxy_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files.
install(DIRECTORY
  launch
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
