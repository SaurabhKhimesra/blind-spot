"""The folding-part act: two UR5e cells in ONE Gazebo world.

    LEFT  cell - the 2001 partition stays on, always
    RIGHT cell - the feature guard decides, every step

    ros2 launch blindspot_arm fold.launch.py                 # Gazebo GUI
    ros2 launch blindspot_arm fold.launch.py gui:=false      # headless
    ros2 launch blindspot_arm fold.launch.py record:=true rate:=120

One world means one physics clock, so the two cells are frame-locked by
construction; two separate simulators would drift apart and the comparison
would be worthless. The servo node starts by itself once the controllers are
up, and the launch ends when the act is complete.

Everything runs on simulation time. `rate` is physics steps per wall second
(500 = real time at the 2 ms step); recording renders two extra 1080p cameras,
so pair record:=true with a lower rate. Only wall-clock duration changes.
"""

import os

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = "blindspot_arm"


def cell(share, side, y):
    """One arm: description, spawn, camera bridge, controllers.

    Returns the actions and the arm-controller spawner, so the servo node can
    be started when it exits (the controller manager does not exist until the
    model is spawned and gz_ros2_control has loaded)."""
    xacro_file = os.path.join(share, "urdf", "ur5e_camera.urdf.xacro")
    ctrl = os.path.join(share, "config", "controllers_%s.yaml" % side)
    desc = ParameterValue(
        Command(["xacro ", xacro_file, " ur_type:=ur5e", " prefix:=", side, "_",
                 " ns:=/", side, " controllers_file:=", ctrl]), value_type=str)

    def spawner(name):
        return Node(package="controller_manager", executable="spawner",
                    namespace=side, output="log",
                    arguments=[name, "--param-file", ctrl,
                               "--controller-manager",
                               "/%s/controller_manager" % side,
                               "--controller-manager-timeout", "120"])

    arm = spawner("arm_controller")
    actions = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             namespace=side, output="log",
             parameters=[{"robot_description": desc, "use_sim_time": True}]),
        TimerAction(period=6.0, actions=[
            Node(package="ros_gz_sim", executable="create", output="log",
                 arguments=["-topic", "/%s/robot_description" % side,
                            "-name", "ur5e_%s" % side,
                            "-x", "0", "-y", str(y), "-z", "0"])]),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             namespace=side, output="log",
             arguments=["/%s_wrist_camera@sensor_msgs/msg/Image[gz.msgs.Image"
                        % side],
             remappings=[("/%s_wrist_camera" % side, "/%s/camera/image" % side)],
             parameters=[{"use_sim_time": True}]),
        TimerAction(period=10.0, actions=[spawner("joint_state_broadcaster")]),
        TimerAction(period=13.0, actions=[arm]),
    ]
    return actions, arm


def generate_launch_description():
    share = get_package_share_directory(PKG)
    world = os.path.join(share, "worlds", "panel_fold.sdf")
    # where Gazebo finds the UR meshes and the gz_ros2_control plugin
    env = {"GZ_SIM_RESOURCE_PATH": os.pathsep.join([
               os.path.dirname(get_package_share_directory("ur_description")),
               share]),
           "GZ_SIM_SYSTEM_PLUGIN_PATH":
               os.path.join(get_package_prefix("gz_ros2_control"), "lib")}
    gui, rate = LaunchConfiguration("gui"), LaunchConfiguration("rate")
    record = LaunchConfiguration("record")

    left, _ = cell(share, "left", 0.0)
    # 2.5 m apart so each cell's film camera sits at the same pose relative to
    # its own cell, with the other cell outside its field of view
    right, right_arm = cell(share, "right", 2.5)

    servo = Node(package="blindspot_cpp", executable="fold_node",
                 name="blindspot_fold",
                 output="screen",
                 parameters=[{"log": LaunchConfiguration("log"),
                              "t_fold": LaunchConfiguration("t_fold"),
                              "t_end": LaunchConfiguration("t_end")}])
    recorder = Node(package="blindspot_cpp", executable="record_node",
                    name="blindspot_record",
                    output="log", condition=IfCondition(record),
                    parameters=[{"out": LaunchConfiguration("out")}])

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rate", default_value="500"),
        DeclareLaunchArgument("t_fold", default_value="20.0",
                              description="sim seconds before the part folds"),
        DeclareLaunchArgument("t_end", default_value="23.5",
                              description="sim seconds when the act ends"),
        DeclareLaunchArgument("log", default_value="/tmp/blindspot_fold_log.csv",
                              description="per-step CSV of everything the "
                                          "servo node knew"),
        DeclareLaunchArgument("record", default_value="false",
                              description="save every film and wrist camera "
                                          "frame, named by sim timestamp"),
        DeclareLaunchArgument("out", default_value="/tmp/blindspot_frames"),

        ExecuteProcess(cmd=["gz", "sim", "-r", "-v", "2", "-z", rate, world],
                       condition=IfCondition(gui), output="screen",
                       additional_env=env),
        ExecuteProcess(cmd=["gz", "sim", "-s", "-r", "-v", "2", "-z", rate,
                            world],
                       condition=UnlessCondition(gui), output="screen",
                       additional_env=env),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             output="log", parameters=[{"use_sim_time": True}],
             arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                        # ROS -> gz, drives the flap hinges
                        "/fold_left@std_msgs/msg/Float64]gz.msgs.Double",
                        "/fold_right@std_msgs/msg/Float64]gz.msgs.Double"]),
        # the film cameras are bridged only when recording, so they cost
        # nothing otherwise (gz renders a camera only if someone subscribes)
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             output="log", parameters=[{"use_sim_time": True}],
             condition=IfCondition(record),
             arguments=["/cine_left@sensor_msgs/msg/Image[gz.msgs.Image",
                        "/cine_right@sensor_msgs/msg/Image[gz.msgs.Image"]),
        *left, *right,
        RegisterEventHandler(OnProcessExit(target_action=right_arm,
                                           on_exit=[recorder, servo])),
        RegisterEventHandler(OnProcessExit(
            target_action=servo, on_exit=[EmitEvent(event=Shutdown(
                reason="act complete"))])),
    ])
