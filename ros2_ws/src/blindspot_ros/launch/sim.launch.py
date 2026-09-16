"""Live simulation: servo loop closed through the guard node, plus RViz.

    ros2 launch blindspot_ros sim.launch.py calibration:=/tmp/ring_cal.json
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        DeclareLaunchArgument("image_view", default_value="true",
                              description="also open rqt_image_view, which "
                                          "shows the camera view in its own "
                                          "window regardless of how RViz "
                                          "docks its Image panel"),
        Node(package="blindspot_ros", executable="sim_node", name="blindspot_sim",
             output="screen"),
        Node(package="blindspot_ros", executable="guard_node",
             name="blindspot_guard", output="screen",
             parameters=[{"calibration": cal}]),
        Node(package="rviz2", executable="rviz2", name="rviz2",
             arguments=["-d", rviz], output="log",
             condition=IfCondition(LaunchConfiguration("rviz"))),
        Node(package="rqt_image_view", executable="rqt_image_view",
             name="rqt_image_view", arguments=["/camera/image"], output="log",
             condition=IfCondition(LaunchConfiguration("image_view"))),
    ])
