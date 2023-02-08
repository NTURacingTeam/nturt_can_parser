# nturt_can_parser

## Introduction

This ros2 package provides bridge between socket can and ros2 message. Though this ros2 package is called can "parser", it don't parse any of the incoming can signals. Instead, [NTURacingTeam/nturt_can_config](https://github.com/NTURacingTeam/nturt_can_config) provide such function using c code generated from dbc(can database) file.

> Obviously, the name "parser" suggests that this package was once used as a parser of can message before we find out that there's a de facto standard dbc format that document such. And even dozens of code generators built for that. Better research before doing any hard works! If you are interested in how this package was used for parsing can message, please checkout `ros1` branch.

### ROS2 SocketCAN

The library used for bridging socket can and ros2 node implementation was adopted from [autowarefoundation/ros2_socketcan](https://github.com/autowarefoundation/ros2_socketcan/tree/0b4c0d9bf7214467d7c520ed2d5dd72806c664a6). Yet the library has minor defect that the receiver node will receive the can message sent by the sender node due to socket can's `local loopback`. Hence we forked the package into our own [NTURacingTeam/ros2_socketcan](https://github.com/NTURacingTeam/ros2_socketcan).

### Real-time support

The sender/receiver node from [autowarefoundation/ros2_socketcan](https://github.com/autowarefoundation/ros2_socketcan) does not  support for real-time. Hence this package adds it's own implementation of the sender/receiver node in order to make them real-time.

## Usage

### configure_can.sh

After system reboot, can bus have to be configured before use, hence this node(process) is used for such purpose, and it exits afterwards.

Usage:

```shell=
ros2 run nturt_can_parser configure_can.sh [OPTIONS]
```

Use `-h` option to checkout its full usage.

### socket_can_receiver

This ros2 node publishes `can_msgs/msg/Frame` message to `from_can_bus` topic when receiving can signal from socket can. To run it in real-time mode, use `--real-time` option.

Usage:

```shell=
ros2 run nturt_can_parser socket_can_receiver [--real-time]
```

### socket_can_sender

This ros2 node subscribes `can_msgs/msg/Frame` message from `to_can_bus` topic to send can signal to socket can. To run it in real-time mode, use `--real-time` option.

Usage:

```shell=
ros2 run nturt_can_parser socket_can_sender [--real-time]
```

### socker_can_bridge.launch.py

This launch file combines the functionality of the above three nodes.

Usage:

```shell=
ros2 launch nturt_can_parser socker_can_bridge.launch.py
```

#### Launch Configuration

There are couple ros2 parameters defined in this launch file:

1. `password` - string: The password of the sudo user for configuring can bus, required.
2. `bitrate` - int: The Bitrate at which the can bus will transfer can signal, default to `100000`(100K).
3. `is_realtime` - bool: Whether to run in real-time mode, default to `true`.

## Tests

This package provides two python scripts to test socket can:

1. can_receive_test.py
2. can_send_test.py

Use `-h` option to checkout theirs full usage.
