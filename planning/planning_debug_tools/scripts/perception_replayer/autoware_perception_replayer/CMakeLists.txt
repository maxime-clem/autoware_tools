cmake_minimum_required(VERSION 3.8)
project(autoware_perception_replayer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
set(CMAKE_AUTOMOC ON)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_components
  rosbag2_cpp
  rosbag2_storage
  geometry_msgs
  sensor_msgs
  autoware_perception_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

# Node implementation
add_library(perception_replayer_node_lib SHARED
  src/perception_replayer_node.cpp
  src/perception_reproducer_node.cpp
)
target_include_directories(perception_replayer_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(perception_replayer_node_lib
  ${dependencies}
)
target_link_libraries(perception_replayer_node_lib
  Qt5::Widgets
  Qt5::Core
)
rclcpp_components_register_nodes(perception_replayer_node_lib
  "autoware_perception_replayer::PerceptionReplayerNode"
  "autoware_perception_replayer::PerceptionReproducerNode"
)

# Executable
add_executable(perception_replayer
  src/main.cpp
)
target_include_directories(perception_replayer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(perception_replayer
  ${dependencies}
)
target_link_libraries(perception_replayer
  perception_replayer_node_lib
  Qt5::Widgets
  Qt5::Core
)

add_executable(perception_reproducer
  src/main.cpp
)
target_include_directories(perception_reproducer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(perception_reproducer
  ${dependencies}
)
target_link_libraries(perception_reproducer
  perception_replayer_node_lib
  Qt5::Widgets
  Qt5::Core
)

install(TARGETS
  perception_replayer_node_lib
  perception_replayer
  perception_reproducer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
