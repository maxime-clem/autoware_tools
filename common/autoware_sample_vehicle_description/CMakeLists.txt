cmake_minimum_required(VERSION 3.14)
project(autoware_sample_vehicle_description)

find_package(autoware_cmake REQUIRED)
find_package(xacro REQUIRED)
autoware_package()

ament_auto_package(INSTALL_TO_SHARE
  mesh
  urdf
  config
)
