#!/usr/bin/env python3
"""
demo.launch.py
--------------
Launches the full block-painting-helper demo stack:
  0. rosbridge          — turtlebot communication
  1. navstack           - SLAM node that works with Turtlebot bridge
  2. person_tracker      — camera, static TF, person tracker, top-down viz
  3. arm.launch.py       — UR3e driver + virtual spring controller + torque relay 4. moveit              - UR3e moveit server
  5. bph_pickmeup_node   — arm pick-and-place action server
  5.5 overhead webcam - v4l2_camera camera node publishing on /image_raw
                         currently loaded separately from different computer
  6. color_picker_node   — overhead-camera colour-based object localisation
  7. simple_sm_node      — top-level SMACH state machine

FIXME: document all the parameters here

Example:
  ros2 launch bph_statemachine demo.launch.py use_sim_time:=false
  ros2 launch bph_statemachine demo.launch.py cam_z:=2.0 
"""

import math
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg     = get_package_share_directory("nav_to_goal")
    tracker_pkg = get_package_share_directory("person_tracker")
    # bph_statemachine shares its launch dir with arm.launch.py
    bph_sm_pkg  = get_package_share_directory("bph_statemachine")
    moveit_pkg = get_package_share_directory("ur_moveit_config")
    nav2_pkg = get_package_share_directory("nav2_bringup")
    
    # ── Inject venv site-packages so all nodes pick up extra dependencies ────
    _venv_site = (
        "/home/katallen/.ros_venv/lib/"
        f"python{sys.version_info.major}.{sys.version_info.minor}"
        "/site-packages"
    )
    set_pythonpath = SetEnvironmentVariable(
        name="PYTHONPATH",
        value=_venv_site + ":" + os.environ.get("PYTHONPATH", ""),
    )

    # ── Shared / navigation args ─────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation clock",
    )

    slam_params_file_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(nav_pkg, "config", "slam_params.yaml"),
        description="slam_toolbox parameter YAML",
    )
    depth_image_topic_arg = DeclareLaunchArgument(
        "depth_image_topic", default_value="/tb-camera/depth/image_raw",
    )
    depth_info_topic_arg = DeclareLaunchArgument(
        "depth_info_topic", default_value="/tb-camera/depth/camera_info",
    )
    robot_base_frame_arg = DeclareLaunchArgument(
        "robot_base_frame", default_value="turtlebot/base_link",
    )

    # ── Overhead camera pose args ────────────────────────────────────────────
    cam_x_arg     = DeclareLaunchArgument("cam_x",     default_value="0.0")
    cam_y_arg     = DeclareLaunchArgument("cam_y",     default_value="0.0")
    cam_z_arg     = DeclareLaunchArgument("cam_z",     default_value="2.5")
    cam_roll_arg  = DeclareLaunchArgument("cam_roll",  default_value="0.0")
    cam_pitch_arg = DeclareLaunchArgument(
        "cam_pitch", default_value=str(math.pi / 2),
    )
    cam_yaw_arg   = DeclareLaunchArgument("cam_yaw",   default_value="0.0")

    # ── UR arm / spring-controller args ─────────────────────────────────────
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="10.3.4.10",
        description="IP address of the UR3e robot.",
    )
    kinematics_params = DeclareLaunchArgument(
        "kinematics_params",
        default_value="/home/katallen/my_robot_calibration.yaml",
        description="Kinematics Calibration file made from ros2 launch ur_calibration calibration_correction.launch.py",
        )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Whether to launch RViz alongside the UR driver.",
    )



    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=PathJoinSubstitution([
            EnvironmentVariable("ROS_WS", default_value="/home/katallen/sandbox"),
            "src/springcontroller/springcontroller/flat_urdf_files/ceeorobot_flat.urdf"
    ]),
        description="Absolute path to the URDF used by the virtual spring node.",
    )

    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="Use fake hardware for UR arm",
    )


    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([EnvironmentVariable("ROS_WS", default_value="/home/katallen/sandbox"),
            "src/block-painting-helper/hayley-nav2/nav2_params.yaml"
    ]),
        description="Parameter file for nav2",
        )
    springconfig_arg = DeclareLaunchArgument(
        "springconfig",
        default_value=PathJoinSubstitution([EnvironmentVariable("ROS_WS", default_value="/home/katallen/sandbox"),
            "src/springcontroller/springcontroller/config/springs.yaml"
    ]),
    )
            

    joint_order_arg = DeclareLaunchArgument(
        "joint_order",
        default_value=(
            "[elbow_joint, shoulder_lift_joint, shoulder_pan_joint,"
            " wrist_1_joint, wrist_2_joint, wrist_3_joint]"
        ),
        description="Ordered joint names for the torque relay.",
    )
    torque_topic_arg = DeclareLaunchArgument(
        "torque_topic",
        default_value="/virtual_spring_node/joint_torques",
        description="Input topic carrying spring torques (sensor_msgs/JointState).",
    )
    command_topic_arg = DeclareLaunchArgument(
        "command_topic",
        default_value="/forward_effort_controller/commands",
        description="Output topic sent to the effort controller.",
    )

    # ── Colour-picker args ───────────────────────────────────────────────────
    color_image_topic_arg = DeclareLaunchArgument(
        "color_image_topic",
        default_value="/bph_overhead_camera/image_raw",
        description=(
            "RGB image topic for the colour-picker node.  "
            "Should be the same physical camera used by person_tracker."
        ),
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── 0. ROSBridge ─────────────────────────────────────────────────────────
    turtlebridge_bringup = Node(
        package="nav_to_goal",
        executable="turtlebot_bridge",
        name="turtle_bridge",
        output="screen",
    )

    # ── 1. Nav2 + SLAM  ──────────────────────────────────────
    nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time":      use_sim_time,
            "params_file":       LaunchConfiguration("params_file"),
        }.items(),
    )

    # IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(nav_pkg, "launch", "bringup.launch.py")
    #     ),
    #     launch_arguments={
    #         "slam_params_file":  LaunchConfiguration("slam_params_file"),
    #         "depth_image_topic": LaunchConfiguration("depth_image_topic"),
    #         "depth_info_topic":  LaunchConfiguration("depth_info_topic"),
    #         "robot_base_frame":  LaunchConfiguration("robot_base_frame"),
    #         "navigate_on_start": "false",   # state machine drives navigation
    #     }.items(),
    # )

    # ── 2. Overhead camera + person tracker + top-down viz ───────────────────
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
            "color_image_topic": LaunchConfiguration("color_image_topic"),
        }.items(),
    )

    # ── 3. Top-level SMACH state machine ─────────────────────────────────────
    state_machine_node = Node(
        package="bph_statemachine",
        executable="simple_sm_node",
        name="robot_fetch_smach",
        output="screen",
    )

    # ── 4. Arm pick-and-place action server ───────────────────────────────────
    pickmeup_node = Node(
        package="bph_pickmeup",
        executable="bph_pickmeup_node",
        name="bph_pickmeup",
        output="screen",
    )

    moveit_bringup = IncludeLaunchDescription(  
         PythonLaunchDescriptionSource(
             os.path.join(moveit_pkg, "launch","ur_moveit.launch.py")),
         launch_arguments={
             "ur_type": "ur3e",
             "robot_ip": LaunchConfiguration("robot_ip"),
             "launch_rviz": LaunchConfiguration("launch_rviz")}.items(),
     )

    # ── 5. UR3e driver + virtual spring + torque relay ───────────────────────
    #
    #  arm.launch.py lives in the same launch/ directory as this file.
    #  It starts: ur_robot_driver, virtual_spring_node, torque_relay.
    #
    arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bph_sm_pkg, "launch", "arm.launch.py")
        ),
        launch_arguments={
            "robot_ip":      LaunchConfiguration("robot_ip"),
            "launch_rviz":   LaunchConfiguration("launch_rviz"),
            "urdf_path":     LaunchConfiguration("urdf_path"),
            "config":        LaunchConfiguration("springconfig"),
            "joint_order":   LaunchConfiguration("joint_order"),
            "torque_topic":  LaunchConfiguration("torque_topic"),
            "command_topic": LaunchConfiguration("command_topic"),
            "kinematics_params": LaunchConfiguration("kinematics_params"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware")
        }.items(),
    )

    
    # ── 6. Colour-based object picker (perception) ───────────────────────────
    #
    #  Node lives in bph_perception package.
    #  It shares the overhead camera RGB + depth streams with person_tracker.
    #  Exposes:
    #    ~/get_target_pose  (service)  — called by the state machine
    #    ~/set_target_color (service)  — lets the SM change colour at runtime
    #    ~/debug_image      (topic)    — annotated image for rviz / debugging
    #
    color_picker_node = Node(
        package="bph_perception",
        executable="color_picker",
        name="color_picker",
        output="screen",
        parameters=[{
            "color_image_topic": LaunchConfiguration("color_image_topic"),
        }],
    )

    return LaunchDescription([
        set_pythonpath,             # must come first
        # declare all args
        use_sim_time_arg,
        params_file_arg,
        slam_params_file_arg,
        depth_image_topic_arg,
        depth_info_topic_arg,
        robot_base_frame_arg,
        cam_x_arg, cam_y_arg, cam_z_arg,
        cam_roll_arg, cam_pitch_arg, cam_yaw_arg,
        robot_ip_arg,
        launch_rviz_arg,
        kinematics_params,
        urdf_path_arg,
        use_fake_hardware_arg,
        springconfig_arg,
        joint_order_arg,
        torque_topic_arg,
        command_topic_arg,
        color_image_topic_arg,


        # 0. rosbridge          — turtlebot communication
        # 1. navstack           - SLAM node that works with Turtlebot bridge
        # 1.5 overhead webcam - v4l2_camera camera node publishing on /image_raw
        #                        currently loaded separately from different comp
        # 2. person_tracker      — camera, static TF, person tracker, top-down viz
        # 3. arm.launch.py       — UR3e driver + virtual spring controller + torque relay 4. moveit              - UR3e moveit server
        # 5. bph_pickmeup_node   — arm pick-and-place action server
        # 6. color_picker_node   — overhead-camera colour-based object localisation
        # 7. simple_sm_node      — top-level SMACH state machine
        

        # start nodes / sub-launches
        LogInfo(msg="[demo] Starting ROSBridge ..."),
        turtlebridge_bringup,

        LogInfo(msg="[demo] Starting Nav2 + SLAM + navigator_node ..."),
        nav_bringup,

        LogInfo(msg="[demo] Check that v4l2_camera is loaded..."),
        #camera_bringup,

        #LogInfo(msg="[demo] Starting person tracker ..."),
        #person_tracker,
        LogInfo(msg="[demo] Skipping person tracker ..."),

        LogInfo(msg="[demo] Starting UR3e arm + spring controller ..."),
        arm_bringup,

        # LogInfo(msg="[demo] Starting UR3e MoveIt ..."),
        # moveit_bringup,   # TODO: fill in moveit_bringup above

        LogInfo(msg="[demo] Starting arm pick-and-place server ..."),
        pickmeup_node,

        LogInfo(msg="[demo] Starting colour-picker perception node ..."),
        color_picker_node,

        LogInfo(msg="[demo] Starting SMACH state machine ..."),
        state_machine_node,
    ])
