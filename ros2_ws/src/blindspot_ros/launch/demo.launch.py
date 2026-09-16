"""Guard node plus synthetic detections, so the whole thing runs with no camera.

    ros2 launch blindspot_ros demo.launch.py calibration:=/abs/path/cal.json
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cal = LaunchConfiguration("calibration")
    return LaunchDescription([
        DeclareLaunchArgument("calibration",
                              description="path to a calibration JSON from "
                                          "python -m blindspot.calibrate"),
        Node(package="blindspot_ros", executable="fake_detections",
             name="fake_detections", output="screen"),
        Node(package="blindspot_ros", executable="guard_node",
             name="blindspot_guard", output="screen",
             parameters=[{"calibration": cal}]),
    ])
