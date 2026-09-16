"""Checkpoint launch: UR5e with a wrist camera, in Gazebo, holding position.

    ros2 launch blindspot_arm arm.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("blindspot_arm")
    xacro_file = os.path.join(pkg, "urdf", "ur5e_camera.urdf.xacro")
    world = os.path.join(pkg, "worlds", "table.sdf")
    ros_prefix = os.environ.get("ROS_DISTRO_PREFIX", "/opt/ros/lyrical")
    ros_share = os.path.join(ros_prefix, "share")
    ros_lib = os.path.join(ros_prefix, "lib")
    controllers_yaml = os.path.join(pkg, "config", "controllers.yaml")
    # must be an explicit string: launch otherwise tries to parse the
    # whole URDF as YAML and fails.
    robot_desc = ParameterValue(
        Command(["xacro ", xacro_file, " ur_type:=ur5e"]), value_type=str)

    gui = LaunchConfiguration("gui")

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),

        ExecuteProcess(
            cmd=["gz", "sim", "-r", "-v", "2", world],
            output="screen",
            # Gazebo resolves model:// URIs and system plugins from its own
            # paths, not from the ROS prefix, so both have to be handed to it
            # explicitly. Without the first the UR meshes do not load; without
            # the second gz_ros2_control never starts and there are no
            # controllers and no joint_states.
            additional_env={
                "GZ_SIM_RESOURCE_PATH": ros_share + os.pathsep + pkg,
                "GZ_SIM_SYSTEM_PLUGIN_PATH": ros_lib,
            }),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen",
             parameters=[{"robot_description": robot_desc,
                          "use_sim_time": True}]),

        # spawn after the world is up
        TimerAction(period=6.0, actions=[
            Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-topic", "robot_description",
                            "-name", "ur5e",
                            "-x", "0", "-y", "0", "-z", "0.0"]),
        ]),

        # bridge: clock, camera image and info
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             output="screen",
             arguments=[
                 "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                 "/wrist_camera@sensor_msgs/msg/Image[gz.msgs.Image",
                 "/wrist_camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
             ],
             remappings=[("/wrist_camera", "/camera/image"),
                         ("/wrist_camera_info", "/camera/camera_info")],
             parameters=[{"use_sim_time": True}]),

        # Use the controller_manager spawner, not `ros2 control
        # load_controller`: the spawner forwards the parameter file to the
        # controller node. Without it the controller starts with an empty
        # `joints` list and fails to initialise.
        TimerAction(period=12.0, actions=[
            Node(package="controller_manager", executable="spawner",
                 arguments=["joint_state_broadcaster",
                            "--param-file", controllers_yaml],
                 output="screen"),
        ]),
        TimerAction(period=16.0, actions=[
            Node(package="controller_manager", executable="spawner",
                 arguments=["arm_controller",
                            "--param-file", controllers_yaml],
                 output="screen"),
        ]),
    ])
