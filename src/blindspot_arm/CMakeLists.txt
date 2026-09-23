cmake_minimum_required(VERSION 3.16)
project(blindspot_arm)

# The UR5e + wrist camera description, the Gazebo world, the controller
# configuration and the launch wiring for the fold act. The nodes are in
# blindspot_cpp; fold_node finds the xacro through this package's share
# directory at runtime.
find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch worlds DESTINATION share/${PROJECT_NAME})

ament_package()
