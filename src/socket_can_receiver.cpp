#include "nturt_can_parser/socket_can_receiver.hpp"

SocketCanReceiver::SocketCanReceiver(const rclcpp::NodeOptions &_options) : Node("socket_can_receiver_node", _options),
    can_pub_(this->create_publisher<can_msgs::msg::Frame>("/from_can_bus", 100)),
    interface_(this->declare_parameter("interface", "can0")),
    use_bus_time_(this->declare_parameter("use_bus_time", false)) {

    RCLCPP_INFO(this->get_logger(), "interface: %s", interface_.c_str());

    double interval_sec = this->declare_parameter("timeout_sec", 0.01);
    interval_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(interval_sec));
    RCLCPP_INFO(this->get_logger(), "interval(s): %f", interval_sec);

    RCLCPP_INFO(this->get_logger(), "use bus time: %d", use_bus_time_);

    // initialize can receiver driver
    try {
        receiver_ = std::make_unique<drivers::socketcan::SocketCanReceiver>(interface_);
    }
    catch(const std::exception &ex) {
        RCLCPP_ERROR(this->get_logger(), "Error opening CAN receiver: %s - %s", interface_.c_str(), ex.what());
        rclcpp::shutdown();
    }

    receiver_thread_ = std::make_unique<std::thread>(&SocketCanReceiver::receive, this);
}

SocketCanReceiver::~SocketCanReceiver() {
    receiver_thread_->join();
}

void SocketCanReceiver::receive() {
    drivers::socketcan::CanId receive_id{};
    can_msgs::msg::Frame frame_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
    frame_msg.header.frame_id = "can";

    while (rclcpp::ok()) {
        try {
            receive_id = receiver_->receive(frame_msg.data.data(), interval_ns_);
        }
        catch(const std::exception &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Error receiving CAN message: %s - %s", interface_.c_str(), ex.what());
        continue;
        }

        if (use_bus_time_) {
            frame_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(receive_id.get_bus_time() * 1000U));
        }
        else {
            frame_msg.header.stamp = this->now();
        }

        frame_msg.id = receive_id.identifier();
        frame_msg.is_rtr = (receive_id.frame_type() == drivers::socketcan::FrameType::REMOTE);
        frame_msg.is_extended = receive_id.is_extended();
        frame_msg.is_error = (receive_id.frame_type() == drivers::socketcan::FrameType::ERROR);
        frame_msg.dlc = receive_id.length();
        can_pub_->publish(std::move(frame_msg));
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(SocketCanReceiver)
