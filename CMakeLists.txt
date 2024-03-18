cmake_minimum_required(VERSION 3.5)

project(mrg_slam_sim)

find_package(ament_cmake REQUIRED)

# ROS2 build
install(DIRECTORY
  config
  models
  worlds
  launch
  DESTINATION share/${PROJECT_NAME}
)

# setup the environmental hooks for the Gazebo resource path
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.dsv.in")

ament_package()
