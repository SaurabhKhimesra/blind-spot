"""Side by side, folding part: two UR5e cells in ONE Gazebo world.

    LEFT  cell - guard OFF: the 2001 partition stays on no matter what
    RIGHT cell - guard ON : the feature guard decides, per step

One world means one physics clock, so the two runs are frame-locked by
construction. Two separate simulators would drift apart and the comparison
would be worthless.

    ros2 launch blindspot_arm fold.launch.py gui:=false rate:=125
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = "blindspot_arm"


def cell(pkg, side, x, ros_share, ros_lib, y=0.0):
    """One arm: description, spawn, controllers, camera bridge."""
    xacro_file = os.path.join(pkg, "urdf", "ur5e_camera.urdf.xacro")
    ctrl = os.path.join(pkg, "config", "controllers_%s.yaml" % side)
    desc = ParameterValue(
        Command(["xacro ", xacro_file,
                 " ur_type:=ur5e",
                 " prefix:=", side, "_",
                 " ns:=/", side,
                 " controllers_file:=", ctrl]), value_type=str)
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             namespace=side, output="log",
             parameters=[{"robot_description": desc, "use_sim_time": True}]),
        TimerAction(period=6.0, actions=[
            Node(package="ros_gz_sim", executable="create", output="log",
                 arguments=["-topic", "/%s/robot_description" % side,
                            "-name", "ur5e_%s" % side,
                            "-x", str(x), "-y", str(y), "-z", "0"]),
        ]),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             namespace=side, output="log",
             arguments=[
                 "/%s_wrist_camera@sensor_msgs/msg/Image[gz.msgs.Image" % side,
                 "/%s_wrist_camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" % side,
             ],
             remappings=[("/%s_wrist_camera" % side, "/%s/camera/image" % side),
                         ("/%s_wrist_camera_info" % side,
                          "/%s/camera/camera_info" % side)],
             parameters=[{"use_sim_time": True}]),
        TimerAction(period=10.0, actions=[
            Node(package="controller_manager", executable="spawner",
                 namespace=side, output="log",
                 arguments=["joint_state_broadcaster",
                            "--param-file", ctrl,
                            "--controller-manager", "/%s/controller_manager" % side,
                            "--controller-manager-timeout", "120"]),
        ]),
        TimerAction(period=13.0, actions=[
            Node(package="controller_manager", executable="spawner",
                 namespace=side, output="log",
                 arguments=["arm_controller",
                            "--param-file", ctrl,
                            "--controller-manager", "/%s/controller_manager" % side,
                            "--controller-manager-timeout", "120"]),
        ]),
    ]


def generate_launch_description():
    pkg = get_package_share_directory(PKG)
    world = os.path.join(pkg, "worlds", "panel_fold.sdf")
    ros_prefix = "/opt/ros/lyrical"
    ros_share, ros_lib = ros_prefix + "/share", ros_prefix + "/lib"
    gui = LaunchConfiguration("gui")
    # physics steps per wall second. 500 is real time at the 2 ms step; the
    # film cameras render 1080p, so recording runs slower than real time.
    # Everything downstream is on sim time, so only wall-clock time changes.
    rate = LaunchConfiguration("rate")
    env = {"GZ_SIM_RESOURCE_PATH": ros_share + os.pathsep + pkg,
           "GZ_SIM_SYSTEM_PLUGIN_PATH": ros_lib}

    actions = [
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rate", default_value="500"),
        ExecuteProcess(cmd=["gz", "sim", "-r", "-v", "2", "-z", rate, world],
                       condition=IfCondition(gui),
                       output="screen", additional_env=env),
        ExecuteProcess(cmd=["gz", "sim", "-s", "-r", "-v", "2", "-z", rate, world],
                       condition=UnlessCondition(gui),
                       output="screen", additional_env=env),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             output="log",
             arguments=[
                 "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                 # ROS -> gz, to drive the flap hinges
                 "/fold_left@std_msgs/msg/Float64]gz.msgs.Double",
                 "/fold_right@std_msgs/msg/Float64]gz.msgs.Double",
                 # gz -> ROS, film cameras (recorded, never seen by control)
                 "/cine_left@sensor_msgs/msg/Image[gz.msgs.Image",
                 "/cine_right@sensor_msgs/msg/Image[gz.msgs.Image",
             ],
             parameters=[{"use_sim_time": True}]),
    ]
    actions += cell(pkg, "left", 0.0, ros_share, ros_lib)
    # 2.5 m apart along y (the duel used 0.9 along x) so each cell's film
    # camera can sit at the SAME pose relative to its cell, side-on, with the
    # other cell outside its field of view
    actions += cell(pkg, "right", 0.0, ros_share, ros_lib, y=2.5)
    return LaunchDescription(actions)
