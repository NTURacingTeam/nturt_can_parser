#include "nturt_can_parser/socket_can_sender.hpp"

SocketCanSender::SocketCanSender(const rclcpp::NodeOptions &_options)
    : Node("socket_can_sender_node", _options),
      can_sub_(this->create_subscription<can_msgs::msg::Frame>(
          "to_can_bus", 100,
          std::bind(&SocketCanSender::onCan, this, std::placeholders::_1))),
      interface_(this->declare_parameter("interface", "can0")) {
  RCLCPP_INFO(this->get_logger(), "interface: %s", interface_.c_str());

  double timeout_sec = this->declare_parameter("timeout_sec", 0.01);
  timeout_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(timeout_sec));
  RCLCPP_INFO(this->get_logger(), "timeout(s): %f", timeout_sec);

  // initialize can sender driver
  try {
    sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(interface_);
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Error opening CAN sender: %s - %s",
                 interface_.c_str(), ex.what());
    rclcpp::shutdown();
  }
}

void SocketCanSender::onCan(const can_msgs::msg::Frame::SharedPtr _msg) {
  drivers::socketcan::FrameType type;
  if (_msg->is_rtr) {
    type = drivers::socketcan::FrameType::REMOTE;
  } else if (_msg->is_error) {
    type = drivers::socketcan::FrameType::ERROR;
  } else {
    type = drivers::socketcan::FrameType::DATA;
  }

  drivers::socketcan::CanId send_id =
      _msg->is_extended
          ? drivers::socketcan::CanId(_msg->id, 0, type,
                                      drivers::socketcan::ExtendedFrame)
          : drivers::socketcan::CanId(_msg->id, 0, type,
                                      drivers::socketcan::StandardFrame);
  try {
    sender_->send(_msg->data.data(), _msg->dlc, send_id, timeout_ns_);
  } catch (const std::exception &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Error sending CAN message: %s - %s",
                         interface_.c_str(), ex.what());
    return;
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(SocketCanSender)
