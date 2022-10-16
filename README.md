# nturt_can_parser

## Introduction

Implement the data parsing between can signal from ros topic `/received_messages` and `sent_messages` from node `socketcan_bridge_node` in ros package [`socketcan_bridge`](https://wiki.ros.org/socketcan_bridge). This node translates the normal 1~8 bytes long data from a can frame into desired data type (mainly in double) with desired resolution and offset as

$$\text{desired data}=\text{resolution}\times\text{raw data}-\text{offset}$$

and translates back vice versa as configured in the "can_config" parameter (more information in the `Can parameter` section below).

Once the can parser receives a can frame, the data is stored in can parser's buffer, in order to get the data or other utilities, please checkout `ROS interface` section below.

## Usage

### Can parameter

To parse raw can frame data into desired data, this node needs a config file (in yaml) specified by ros `can_config` parameter. For more information how to wirte the config file, please checkout `README.md` file in the `doc` directory.

### socket_can_bridge

This node requires `socketcan_bridge_node` in ros package [`socketcan_bridge`](https://wiki.ros.org/socketcan_bridge) that transmits and receives raw can data into ros messages in topic `/received_messages` and `sent_messages` (message published on `/received_messages` when received can frame and publish to `sent_messages` for transmit can frame).

**Please also run `socketcan_bridge_node` when using this node**.

### ROS interfaces

This node uses the following ros topics and services to transfer data to other nodes.

#### `/publish_can_frame`

Publish a can frame by it's name.

Message type: `String` in `std_msgs`.

Publish the name of the can frame to be published (as `data` in `String`) to this topic in order to publish this can frame to can bus. The published can data is the data stored in can parser's buffer.

#### `/update_can_data`

Update a can data by it's name.

Message type `UpdateCanData` in `nturt_ros_interface`.

Publish the name of the can data to be updated (as `name` in `UpdateCanData`) and the updated data (as `data` in `UpdateCanData`) to this topic in order to change the can data stored in can parser's buffer.

**Can parser does not check for data overflowing, so plase be aware of whether the data will overflow when parsed into raw can data.**

#### `/get_can_data`

Get the can data stroed in can parser's buffer by it's name.

Service type: `GetCanData` in `nturt_ros_interface`.

Call the name of the can data to get data (as `name` in `GetCanData`)
to this service, can parser will return the can data's data stored in can parser (as `data` in `GetCanData`).

#### `/register_can_notification`

Register to can parser what data to published to a specific topic, when that registered data got updated by received can signals.

**Caution: Please wait until this service is brocasted before calling it by using [`ros::service::waitForService()`](https://docs.ros.org/en/api/roscpp/html/namespaceros_1_1service.html#aabe996581255345b3383e66eaaedef5a).**

Service type: `RegisterCanNotification` in `nturt_ros_interface`.

Call a variable sized array of the name of the can data to register (as `data_name` in `RegisterCanNotification`) and the name of the node registering (as `node_name` in `RegisterCanNotification`), can parser will return the topic name the registering node should subscribed to  (as `topic_name` in `RegisterCanNotification`).

The message type of the notification is `UpdateCanData` in `nturt_ros_interface`, it works the same as in `/update_can_data` section above.
