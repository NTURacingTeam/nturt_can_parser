from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node

# conditional sustitution for realtime node argument
def _realtime_command(condition):
    cmd = ['"--realtime" if "true" == "', condition, '" else ""']
    return PythonExpression(cmd)

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
    arguments.append(
        DeclareLaunchArgument(
            "bitrate",
            default_value="100000",
            description="The Bitrate at which the can bus will transfer can signal.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "is_realtime",
            default_value="true",
            description="Arguement to determine whether to run in real-time.",
        )
    )

    # initialize arguments
    password = LaunchConfiguration("password")
    bitrate = LaunchConfiguration("bitrate")
    is_realtime = LaunchConfiguration("is_realtime")

    # declare node
    # node for configuring can bus
    configure_can_node = Node(
        package="nturt_can_parser",
        executable="configure_can.sh",
        arguments=[
            "-p",
            password,
            "-b",
            bitrate
        ],
        output="both",
    )
    socket_can_receiver_node = Node(
        package="nturt_can_parser",
        executable="socket_can_receiver_node",
        arguments=[
            _realtime_command(is_realtime),
        ],
        output="both",
    )
    socket_can_sender_node = Node(
        package="nturt_can_parser",
        executable="socket_can_sender_node",
        arguments=[
            _realtime_command(is_realtime),
        ],
        output="both",
    )

    # declare event handler
    delay_after_configure_can_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=configure_can_node,
            on_exit=[
                socket_can_receiver_node,
                socket_can_sender_node,
            ],
        )
    )

    return LaunchDescription(
        arguments + [
            configure_can_node,
            delay_after_configure_can_node,
        ]
    )
