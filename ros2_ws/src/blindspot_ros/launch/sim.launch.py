"""Live simulation: servo loop closed through the guard node, plus RViz.

    ros2 launch blindspot_ros sim.launch.py calibration:=/tmp/ring_cal.json
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cal = LaunchConfiguration("calibration")
    rviz = os.path.join(get_package_share_directory("blindspot_ros"),
                        "rviz", "guard.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("calibration",
                              description="calibration JSON from "
                                          "python -m blindspot.calibrate"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="blindspot_ros", executable="sim_node", name="blindspot_sim",
             output="screen"),
        Node(package="blindspot_ros", executable="guard_node",
             name="blindspot_guard", output="screen",
             parameters=[{"calibration": cal}]),
        Node(package="rviz2", executable="rviz2", name="rviz2",
             arguments=["-d", rviz], output="log",
             condition=__import__("launch.conditions", fromlist=["IfCondition"])
             .IfCondition(LaunchConfiguration("rviz"))),
    ])
