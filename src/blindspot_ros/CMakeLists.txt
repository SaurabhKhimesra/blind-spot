cmake_minimum_required(VERSION 3.16)
project(blindspot_ros)

# Launch wiring and the RViz layout for the guard node, the servo simulation
# and the synthetic detector. The nodes themselves are in blindspot_cpp.
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_package()
