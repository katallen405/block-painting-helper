#!/usr/bin/env python3
"""
demo.launch.py
--------------
Launches the full block-painting-helper demo stack:

  1. nav_to_goal/bringup  — depth->scan, slam_toolbox, Nav2, navigator_node
  2. person_tracker       — camera, static TF, person tracker, top-down viz
  3. simple_sm_node       — top-level SMACH state machine
  4. bph_pickmeup_node    — arm pick-and-place action server

Key arguments (all optional):
  use_sim_time      : bool  (default false)
  params_file       : path  (default nav_to_goal config/nav2_params.yaml)
  slam_params_file  : path  (default nav_to_goal config/slam_params.yaml)
  depth_image_topic : str   (default /camera/depth/image_raw)
  depth_info_topic  : str   (default /camera/depth/camera_info)
  robot_base_frame  : str   (default base_link)
  cam_x / cam_y / cam_z / cam_roll / cam_pitch / cam_yaw : float
    (overhead camera pose in the map frame)

Example:
  ros2 launch bph_statemachine demo.launch.py use_sim_time:=false
  ros2 launch bph_statemachine demo.launch.py cam_z:=2.0 cam_x:=1.5 cam_y:=0.5 cam_pitch:=1.0
"""

import os
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg     = get_package_share_directory("nav_to_goal")
    tracker_pkg = get_package_share_directory("person_tracker")

    # ---- Shared args ---------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation clock",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(nav_pkg, "config", "nav2_params.yaml"),
        description="Nav2 parameter YAML",
    )
    slam_params_file_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(nav_pkg, "config", "slam_params.yaml"),
        description="slam_toolbox parameter YAML",
    )
    depth_image_topic_arg = DeclareLaunchArgument(
        "depth_image_topic", default_value="/camera/depth/image_raw",
    )
    depth_info_topic_arg = DeclareLaunchArgument(
        "depth_info_topic", default_value="/camera/depth/camera_info",
    )
    robot_base_frame_arg = DeclareLaunchArgument(
        "robot_base_frame", default_value="base_link",
    )

    cam_x_arg     = DeclareLaunchArgument("cam_x",     default_value="0.0")
    cam_y_arg     = DeclareLaunchArgument("cam_y",     default_value="0.0")
    cam_z_arg     = DeclareLaunchArgument("cam_z",     default_value="2.5")
    cam_roll_arg  = DeclareLaunchArgument("cam_roll",  default_value="0.0")
    cam_pitch_arg = DeclareLaunchArgument("cam_pitch", default_value=str(math.pi / 2))
    cam_yaw_arg   = DeclareLaunchArgument("cam_yaw",   default_value="0.0")

    use_sim_time = LaunchConfiguration("use_sim_time")

    # ---- 1. Nav2 + SLAM + navigator_node ------------------------------------
    nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "use_sim_time":      use_sim_time,
            "params_file":       LaunchConfiguration("params_file"),
            "slam_params_file":  LaunchConfiguration("slam_params_file"),
            "depth_image_topic": LaunchConfiguration("depth_image_topic"),
            "depth_info_topic":  LaunchConfiguration("depth_info_topic"),
            "robot_base_frame":  LaunchConfiguration("robot_base_frame"),
            "navigate_on_start": "false",  # state machine drives navigation
        }.items(),
    )

    # ---- 2. Camera + person tracker + top-down viz --------------------------
    person_tracker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tracker_pkg, "launch", "person_tracker.launch.py")
        ),
        launch_arguments={
            "cam_x":     LaunchConfiguration("cam_x"),
            "cam_y":     LaunchConfiguration("cam_y"),
            "cam_z":     LaunchConfiguration("cam_z"),
            "cam_roll":  LaunchConfiguration("cam_roll"),
            "cam_pitch": LaunchConfiguration("cam_pitch"),
            "cam_yaw":   LaunchConfiguration("cam_yaw"),
        }.items(),
    )

    # ---- 3. Top-level SMACH state machine -----------------------------------
    state_machine_node = Node(
        package="bph_statemachine",
        executable="simple_sm_node",
        name="robot_fetch_smach",
        output="screen",
    )

    # ---- 4. Arm pick-and-place action server --------------------------------
    pickmeup_node = Node(
        package="bph_pickmeup",
        executable="bph_pickmeup_node",
        name="bph_pickmeup",
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        slam_params_file_arg,
        depth_image_topic_arg,
        depth_info_topic_arg,
        robot_base_frame_arg,
        cam_x_arg, cam_y_arg, cam_z_arg,
        cam_roll_arg, cam_pitch_arg, cam_yaw_arg,
        LogInfo(msg="[demo] Starting Nav2 + SLAM + navigator_node ..."),
        nav_bringup,
        LogInfo(msg="[demo] Starting person tracker ..."),
        person_tracker,
        LogInfo(msg="[demo] Starting state machine ..."),
        state_machine_node,
        LogInfo(msg="[demo] Starting arm action server ..."),
        pickmeup_node,
    ])
