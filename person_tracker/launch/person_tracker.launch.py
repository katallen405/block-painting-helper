"""
person_tracker.launch.py
Launch the person tracker node with parameters loaded from a YAML config file.

Usage:
  ros2 launch person_tracker person_tracker.launch.py
  ros2 launch person_tracker person_tracker.launch.py config:=<path/to/config.yaml>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("person_tracker"), "config", "params.yaml"]
        ),
        description="Path to parameter YAML file",
    )

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        parameters=[{
            'video_device': '/dev/video0',  # change if needed
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link',
        }],
        remappings=[
            ('/image_raw', '/camera/color/image_raw'),
        ],
    )

    return LaunchDescription([config_arg, camera_node, tracker_node])

    tracker_node = Node(
        package="person_tracker",
        executable="person_tracker_node",
        name="person_tracker",
        output="screen",
        parameters=[LaunchConfiguration("config")],
        remappings=[
            # Remap these to match your actual camera topic names:
            # ("/camera/color/image_raw",       "/your_cam/rgb/image_raw"),
            # ("/camera/depth/image_rect_raw",  "/your_cam/depth/image_raw"),
            # ("/camera/depth/camera_info",     "/your_cam/depth/camera_info"),
        ],
    )

    return LaunchDescription([config_arg, tracker_node])
