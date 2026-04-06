"""
person_tracker.launch.py
Launch the camera, static TF, person tracker, and top-down visualizer.

Usage:
  ros2 launch person_tracker person_tracker.launch.py
  ros2 launch person_tracker person_tracker.launch.py config:=<path/to/config.yaml>

Camera transform parameters (camera_link relative to map frame):
  cam_x     (float) X position in metres.        Default: 0.0
  cam_y     (float) Y position in metres.        Default: 0.0
  cam_z     (float) Height in metres.            Default: 2.5
  cam_roll  (float) Roll  in radians.            Default: 0.0
  cam_pitch (float) Pitch in radians (tilt down: positive values). Default: 1.5708 (90deg, straight down)
  cam_yaw   (float) Yaw   in radians.            Default: 0.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import math


def generate_launch_description():

    # ---- Config file -----------------------------------------------------
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("person_tracker"), "config", "params.yaml"]
        ),
        description="Path to parameter YAML file",
    )

    # ---- Viz toggle ------------------------------------------------------
    viz_arg = DeclareLaunchArgument(
        "viz", default_value="true",
        description="Launch the top-down visualizer node",
    )

    # ---- Camera pose args ------------------------------------------------
    cam_x_arg     = DeclareLaunchArgument("cam_x",     default_value="0.0")
    cam_y_arg     = DeclareLaunchArgument("cam_y",     default_value="0.0")
    cam_z_arg     = DeclareLaunchArgument("cam_z",     default_value="2.5",
                        description="Camera height in metres")
    cam_roll_arg  = DeclareLaunchArgument("cam_roll",  default_value="0.0")
    cam_pitch_arg = DeclareLaunchArgument("cam_pitch",
                        default_value=str(math.pi / 2),
                        description="Camera tilt in radians (pi/2 = straight down)")
    cam_yaw_arg   = DeclareLaunchArgument("cam_yaw",   default_value="0.0")

    # ---- Nodes -----------------------------------------------------------
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_tf",
        # args order: x y z yaw pitch roll parent_frame child_frame
        arguments=[
            LaunchConfiguration("cam_x"),
            LaunchConfiguration("cam_y"),
            LaunchConfiguration("cam_z"),
            LaunchConfiguration("cam_yaw"),
            LaunchConfiguration("cam_pitch"),
            LaunchConfiguration("cam_roll"),
            "map",
            "camera_link",
        ],
    )

    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera",
        parameters=[{
            "video_device": "/dev/video0",
            "image_size": [640, 480],
            "camera_frame_id": "camera_link",
        }],
        remappings=[
            ("/image_raw", "/camera/color/image_raw"),
        ],
    )

    tracker_node = Node(
        package="person_tracker",
        executable="person_tracker_node",
        name="person_tracker",
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )

    viz_node = Node(
        package="person_tracker",
        executable="topdown_viz_node",
        name="topdown_viz",
        output="screen",
        parameters=[LaunchConfiguration("config")],
        condition=IfCondition(LaunchConfiguration("viz")),
    )

    return LaunchDescription([
        config_arg,
        viz_arg,
        cam_x_arg, cam_y_arg, cam_z_arg,
        cam_roll_arg, cam_pitch_arg, cam_yaw_arg,
        static_tf_node,
        camera_node,
        tracker_node,
        viz_node,
    ])
