#!/usr/bin/env python3
"""
bringup.launch.py
-----------------
Launches the full stack for depth-camera SLAM + Nav2 navigation:

  1. depthimage_to_laserscan   — converts depth image → LaserScan
  2. slam_toolbox (async)      — builds the map in real time
  3. nav2_bringup              — full Nav2 lifecycle stack
  4. navigator_node            — sends the goal once everything is ready

Key arguments (all optional):
  use_sim_time    : bool   (default false)
  params_file     : path   (default config/nav2_params.yaml)
  slam_params_file: path   (default config/slam_params.yaml)
  goal_x          : float  (default 2.0)
  goal_y          : float  (default 1.0)
  goal_yaw        : float  (default 0.0)
  depth_image_topic : str  (default /camera/depth/image_raw)
  depth_info_topic  : str  (default /camera/depth/camera_info)
  robot_base_frame  : str  (default base_link)

Example – TurtleBot4:
  ros2 launch nav_to_goal bringup.launch.py \
      depth_image_topic:=/oakd/rgb/preview/depth \
      robot_base_frame:=base_link \
      goal_x:=3.0 goal_y:=2.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = get_package_share_directory("nav_to_goal")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    # ── Launch arguments ───────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation (Gazebo) clock"
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_dir, "config", "nav2_params.yaml"),
        description="Full path to Nav2 parameter YAML"
    )
    slam_params_file_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(pkg_dir, "config", "slam_params.yaml"),
        description="Full path to slam_toolbox parameter YAML"
    )
    goal_x_arg = DeclareLaunchArgument("goal_x", default_value="2.0")
    goal_y_arg = DeclareLaunchArgument("goal_y", default_value="1.0")
    goal_yaw_arg = DeclareLaunchArgument("goal_yaw", default_value="0.0")
    depth_image_topic_arg = DeclareLaunchArgument(
        "depth_image_topic",
        default_value="/camera/depth/image_raw",
        description="Topic published by the depth camera driver"
    )
    depth_info_topic_arg = DeclareLaunchArgument(
        "depth_info_topic",
        default_value="/camera/depth/camera_info",
        description="CameraInfo topic for the depth camera"
    )
    robot_base_frame_arg = DeclareLaunchArgument(
        "robot_base_frame", default_value="base_link"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")

    # ── 1. depthimage_to_laserscan ─────────────────────────────────────────
    depth_to_scan_node = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depth_to_laserscan",
        parameters=[{
            "scan_time": 0.033,
            "range_min": 0.15,
            "range_max": 3.5,
            "scan_height": 5,        # rows to collapse into the scan
            "output_frame": "base_link",  # overridden per robot if needed
            "use_sim_time": use_sim_time,
        }],
        remappings=[
            ("image", LaunchConfiguration("depth_image_topic")),
            ("camera_info", LaunchConfiguration("depth_info_topic")),
            ("scan", "/scan"),
        ],
    )

    # ── 2. slam_toolbox ────────────────────────────────────────────────────
    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
    )

    # ── 3. Nav2 bringup (no AMCL — SLAM provides localisation) ────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "use_lifecycle_mgr": "true",
            "use_composition": "false",
        }.items(),
    )

    # ── 4. Navigator goal node ─────────────────────────────────────────────
    navigator_node = Node(
        package="nav_to_goal",
        executable="navigator_node",
        name="navigator_node",
        output="screen",
        parameters=[{
            "goal_x": LaunchConfiguration("goal_x"),
            "goal_y": LaunchConfiguration("goal_y"),
            "goal_yaw": LaunchConfiguration("goal_yaw"),
            "robot_base_frame": LaunchConfiguration("robot_base_frame"),
            "use_sim_time": use_sim_time,
            "navigate_on_start": True,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        slam_params_file_arg,
        goal_x_arg,
        goal_y_arg,
        goal_yaw_arg,
        depth_image_topic_arg,
        depth_info_topic_arg,
        robot_base_frame_arg,
        LogInfo(msg="Starting depth→scan converter …"),
        depth_to_scan_node,
        LogInfo(msg="Starting slam_toolbox …"),
        slam_node,
        LogInfo(msg="Starting Nav2 …"),
        nav2_launch,
        LogInfo(msg="Starting navigator goal node …"),
        navigator_node,
    ])
