"""Live simulation: a servo loop closed through the guard node, shown in RViz.

    ros2 launch blindspot_ros sim.launch.py
    ros2 launch blindspot_ros sim.launch.py rviz:=false      # headless

Every step the control law is chosen by the Bool on
/blindspot_guard/partition_ok: the 2001 partitioned law when True, the plain
truncated law when False. The scene cycles healthy -> occluded to two markers
-> collapsed target, re-posing the camera so each regime is a real approach.
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _calibration(given):
    """Path to a calibration: the one given, or the example ring calibrated now.

    `ros2 run blindspot_cpp example_target` prints the path on its last line.
    """
    out = subprocess.run(
        ["ros2", "run", "blindspot_cpp", "example_target"] + ([given] if given else []),
        capture_output=True, text=True, check=True)
    return out.stdout.strip().splitlines()[-1]


def nodes(context):
    given = LaunchConfiguration("calibration").perform(context)
    cal = _calibration(given)
    rviz = os.path.join(get_package_share_directory("blindspot_ros"),
                        "rviz", "guard.rviz")
    return [
        LogInfo(msg="guard calibration: %s%s" % (
            cal, "" if given else "  (known-good example ring)")),
        Node(package="blindspot_cpp", executable="sim_node",
             name="blindspot_sim", output="screen"),
        Node(package="blindspot_cpp", executable="guard_node",
             name="blindspot_guard", output="screen",
             parameters=[{"calibration": cal}]),
        Node(package="rviz2", executable="rviz2", name="rviz2",
             arguments=["-d", rviz], output="log",
             condition=IfCondition(LaunchConfiguration("rviz"))),
        Node(package="rqt_image_view", executable="rqt_image_view",
             name="rqt_image_view", arguments=["/camera/image"], output="log",
             condition=IfCondition(LaunchConfiguration("image_view"))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("calibration", default_value="",
                              description="calibration JSON; empty = calibrate "
                                          "the example ring on start-up"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("image_view", default_value="true",
                              description="also open rqt_image_view on the "
                                          "camera image"),
        OpaqueFunction(function=nodes),
    ])
