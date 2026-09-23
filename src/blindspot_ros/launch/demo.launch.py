"""Guard node plus synthetic detections: the whole decision path, no camera.

    ros2 launch blindspot_ros demo.launch.py
    ros2 launch blindspot_ros demo.launch.py calibration:=/abs/path/cal.json

The detections cycle healthy -> occluded to two markers -> collapsed target,
and are shuffled every frame so the ordering logic is exercised, not assumed.
"""

import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
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
    return [
        LogInfo(msg="guard calibration: %s%s" % (
            cal, "" if given else "  (known-good example ring)")),
        Node(package="blindspot_cpp", executable="fake_detections",
             name="fake_detections", output="screen"),
        Node(package="blindspot_cpp", executable="guard_node",
             name="blindspot_guard", output="screen",
             parameters=[{"calibration": cal}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("calibration", default_value="",
                              description="calibration JSON; empty = calibrate "
                                          "the example ring on start-up"),
        OpaqueFunction(function=nodes),
    ])
