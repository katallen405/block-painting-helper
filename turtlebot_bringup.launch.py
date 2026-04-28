from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ── External launch files ──────────────────────────────────────────────

    kobuki_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("kobuki_node"),
                "launch",
                "kobuki_node-launch.py",
            )
        )
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("openni2_camera"),
                "launch",
                "camera_only.launch.py",
            )
        )
    )

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rosbridge_server"),
                "launch",
                "rosbridge_websocket_launch.xml",
            )
        )
    )

    # ── Static transforms ──────────────────────────────────────────────────

    # base_footprint → openni_depth_optical_frame
    depth_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="depth_optical_frame_publisher",
        arguments=["0.1", "0.0", "0.2", "0.0", "0.0", "0.0",
                   "base_footprint", "openni_depth_optical_frame"],
    )

    # base_footprint → openni_rgb_optical_frame
    rgb_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="rgb_optical_frame_publisher",
        arguments=["0.1", "0.0", "0.2", "0.0", "0.0", "0.0",
                   "base_footprint", "openni_rgb_optical_frame"],
    )

    # ── depthimage_to_laserscan ────────────────────────────────────────────

    depth_to_scan = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depthimage_to_laserscan_node",
        remappings=[
            ("depth",            "/camera/depth/image_raw"),
            ("depth_camera_info", "/camera/depth/camera_info"),
        ],
        parameters=[{
            "scan_time":    0.033,
            "range_min":    0.15,
            "range_max":    10.0,
            "output_frame": "base_footprint",
            "scan_height":  240,
        }],
    )

    # ── Assemble ───────────────────────────────────────────────────────────

    return LaunchDescription([
        kobuki_launch,
        camera_launch,
        depth_tf,
        rgb_tf,
        depth_to_scan,
        rosbridge_launch,
    ])
