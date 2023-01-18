from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # declare arguments
    arguments = []
    arguments.append(
        DeclareLaunchArgument(
            "password",
            default_value="",
            description="The password of the user to get root permission using sudo to setup can bus.",
        )
    )

    # initialize arguments
    password = LaunchConfiguration("password")

    # declare include files
    # node for receiving can signal
    socket_can_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros2_socketcan"),
                "launch",
                "socket_can_receiver.launch.py",
            ]),
        ]),
    )
    # node for sending can signal
    socket_can_sender = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros2_socketcan"),
                "launch",
                "socket_can_sender.launch.py",
            ]),
        ]),
    )

    # declare node
    # node for configuring can bus
    configure_can_node = Node(
        package="nturt_can_parser",
        executable="configure_can.sh",
        arguments=[
            password,
        ],
        output="both",
    )

    # declare event handler
    delay_after_configure_can_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=configure_can_node,
            on_exit=[
                socket_can_receiver,
                socket_can_sender,
            ],
        )
    )

    return LaunchDescription(
        arguments + [
            configure_can_node,
            delay_after_configure_can_node,
        ]
    )
