from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ---------------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------------

    # --- UR robot driver ---
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="10.3.4.10",
        description="IP address of the UR3e robot.",
    )

    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Whether to launch RViz alongside the UR driver.",
    )

    # --- virtual_spring_node ---
    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value="/home/katallen/workspace/src/springcontroller/springcontroller/flat_urdf_files/ceeorobot_flat.urdf",
        description="Absolute path to the URDF file used by the virtual spring node.",
    )

    # --- torque_relay ---
    joint_order_arg = DeclareLaunchArgument(
        "joint_order",
        default_value="[elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]",
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
        description="Output topic sent to the effort controller (std_msgs/Float64MultiArray).",
    )

    # ---------------------------------------------------------------------------
    # UR3e robot driver (included launch file)
    # ---------------------------------------------------------------------------
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "launch",
                "ur3e.launch.py",
            ])
        ]),
        launch_arguments={
            "robot_ip":    LaunchConfiguration("robot_ip"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
        }.items(),
    )

    # ---------------------------------------------------------------------------
    # Virtual spring node
    # ---------------------------------------------------------------------------
    virtual_spring_node = Node(
        package="springcontroller",
        executable="virtual_spring_node",
        name="virtual_spring_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"urdf_path": LaunchConfiguration("urdf_path")}
        ],
    )

    # ---------------------------------------------------------------------------
    # Torque relay (included launch file)
    # ---------------------------------------------------------------------------
    torque_relay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("springcontroller"),
                "launch",
                "torque_relay.launch.py",
            ])
        ]),
        launch_arguments={
            "joint_order":    LaunchConfiguration("joint_order"),
            "torque_topic":   LaunchConfiguration("torque_topic"),
            "command_topic":  LaunchConfiguration("command_topic"),
        }.items(),
    )

    # ---------------------------------------------------------------------------
    # Assemble
    # ---------------------------------------------------------------------------
    return LaunchDescription([
        # arguments
        robot_ip_arg,
        launch_rviz_arg,
        urdf_path_arg,
        joint_order_arg,
        torque_topic_arg,
        command_topic_arg,
        # nodes / included launches
        ur_driver,
        virtual_spring_node,
        torque_relay,
    ])
