/**
 * @file socket_can_sender_node.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief ROS2 node wrapping socket can for sending can signals.
 * 
 * The c++ socket can driver and ros2 node implementation are adopted from
 * [autowarefoundation/ros2_socketcan](https://github.com/autowarefoundation/ros2_socketcan).
 */

#ifndef SOCKET_CAN_SENDER_NODE_HPP
#define SOCKET_CAN_SENDER_NODE_HPP

// std include
#include <memory>
#include <string>

// ros2 include
#include "rclcpp/rclcpp.hpp"

// ros2 message include
#include "can_msgs/msg/frame.hpp"

// ros2_socketcan include
#include "ros2_socketcan/socket_can_sender.hpp"

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for sending can signals to socket can.
 */
class SocketCanSender : public rclcpp::Node {
    public:
        /**
         * @brief Constructor.
         * @param _options Options for node initialization.
         */
        SocketCanSender(const rclcpp::NodeOptions &_options);

    private:
        /// @brief ROS2 subscriber to "/to_can_bus", for receiving can signals that other nodes want to send.
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

        /// @brief Pointer to the socket can driver.
        std::unique_ptr<drivers::socketcan::SocketCanSender> sender_;
        
        // internal states
        /// @brief The name of the socket can interface.
        std::string interface_;

        /// @brief Socket can timout \f$[ns]\f$.
        std::chrono::nanoseconds timeout_ns_;

        /// @brief Callback function when receiving message from "/from_can_bus".
        void onCan(const can_msgs::msg::Frame::SharedPtr _msg);
};

#endif // SOCKET_CAN_SENDER_NODE_HPP
