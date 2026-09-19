"""Guard node plus synthetic detections: the whole decision path, no camera.

    ros2 launch blindspot_ros demo.launch.py
    ros2 launch blindspot_ros demo.launch.py calibration:=/abs/path/cal.json

The detections cycle healthy -> occluded to two markers -> collapsed target,
and are shuffled every frame so the ordering logic is exercised, not assumed.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from blindspot_ros.example_target import ensure_calibration


def nodes(context):
    given = LaunchConfiguration("calibration").perform(context)
    cal = ensure_calibration(given)
    return [
        LogInfo(msg="guard calibration: %s%s" % (
            cal, "" if given else "  (known-good example ring)")),
        Node(package="blindspot_ros", executable="fake_detections",
             name="fake_detections", output="screen"),
        Node(package="blindspot_ros", executable="guard_node",
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
