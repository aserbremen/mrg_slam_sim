cmake_minimum_required(VERSION 3.5)

project(mrg_sim)

find_package(ament_cmake REQUIRED)

# ROS2 build
install(DIRECTORY
  config
  models
  worlds
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
