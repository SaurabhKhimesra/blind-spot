cmake_minimum_required(VERSION 3.16)
project(blindspot_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 Lyrical removed ament_target_dependencies(), so everything below links
# the exported CMake targets directly: rclcpp::rclcpp, ${<pkg>_TARGETS} for
# message packages, urdf::urdf, and so on.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(urdf REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ---------------------------------------------------------------------------
# the library: the guard, the control law, calibration, kinematics, detection
# ---------------------------------------------------------------------------
add_library(blindspot SHARED
  src/units.cpp
  src/guard.cpp
  src/ros_bridge.cpp
  src/arm_law.cpp
  src/calibrate.cpp
  src/kin.cpp
  src/detect.cpp
)
target_include_directories(blindspot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(blindspot PUBLIC
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
  urdf::urdf
  opencv_core
  opencv_imgproc
)

# ---------------------------------------------------------------------------
# executables
# ---------------------------------------------------------------------------
add_executable(calibrate src/calibrate_main.cpp)
target_link_libraries(calibrate PRIVATE blindspot)

add_executable(example_target src/example_target.cpp)
target_link_libraries(example_target PRIVATE blindspot)

add_executable(quickstart src/quickstart.cpp)
target_link_libraries(quickstart PRIVATE blindspot)

add_executable(guard_node src/guard_node.cpp)
target_link_libraries(guard_node PRIVATE
  blindspot
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
)

add_executable(fake_detections src/fake_detections.cpp)
target_link_libraries(fake_detections PRIVATE
  blindspot
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${vision_msgs_TARGETS}
)

add_executable(sim_node src/sim_node.cpp)
target_link_libraries(sim_node PRIVATE
  blindspot
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

add_executable(fold_node src/fold_node.cpp)
target_link_libraries(fold_node PRIVATE
  blindspot
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
)

add_executable(record_node src/record_node.cpp)
target_link_libraries(record_node PRIVATE
  blindspot
  rclcpp::rclcpp
  opencv_imgcodecs
  ${sensor_msgs_TARGETS}
)

add_executable(api_checks src/api_checks.cpp)
target_link_libraries(api_checks PRIVATE blindspot ${vision_msgs_TARGETS})
# The suite drives the calibrate CLI as a subprocess, so hand it the path
# rather than relying on it being installed and on PATH.
target_compile_definitions(api_checks PRIVATE
  BLINDSPOT_CALIBRATE_BIN="$<TARGET_FILE:calibrate>")
add_dependencies(api_checks calibrate)

add_executable(arm_checks src/arm_checks.cpp)
target_link_libraries(arm_checks PRIVATE blindspot)

# ---------------------------------------------------------------------------
# install
# ---------------------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include)
install(TARGETS blindspot
  EXPORT export_blindspot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
  calibrate example_target quickstart fake_detections record_node
  guard_node sim_node fold_node api_checks arm_checks
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  # Both suites report by exit code; GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  # turns that into the JUnit file colcon expects, so neither binary has to
  # know anything about XML.
  ament_add_test(api_checks
    COMMAND "${CMAKE_CURRENT_BINARY_DIR}/api_checks"
    TIMEOUT 120
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  ament_add_test(arm_checks
    COMMAND "${CMAKE_CURRENT_BINARY_DIR}/arm_checks"
    TIMEOUT 300
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(blindspot)
ament_export_targets(export_blindspot HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp urdf vision_msgs Eigen3 nlohmann_json OpenCV)
ament_package()
