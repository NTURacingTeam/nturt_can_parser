/**
 * @file socket_can_receiver_node.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief ROS2 node wrapping socket can for receiving can signals.
 * 
 * The c++ socket can driver and ros2 node implementation are adopted from
 * [autowarefoundation/ros2_socketcan](https://github.com/autowarefoundation/ros2_socketcan).
 */

#ifndef SOCKET_CAN_RECEIVER_NODE_HPP
#define SOCKET_CAN_RECEIVER_NODE_HPP

// std include
#include <memory>
#include <string>
#include <thread>

// ros2 include
#include "rclcpp/rclcpp.hpp"

// ros2 message include
#include "can_msgs/msg/frame.hpp"

// ros2_socketcan include
#include "ros2_socketcan/socket_can_receiver.hpp"

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for receiving can signals from socket can.
 */
class SocketCanReceiver : public rclcpp::Node {
    public:
        /**
         * @brief Constructor.
         * @param _options Options for node initialization.
         */
        SocketCanReceiver(const rclcpp::NodeOptions &_options);

        /// @brief Destructor.
        ~SocketCanReceiver();

    private:
        /// @brief ROS2 publisher to "/from_can_bus", for sending received can signals to other nodes.
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;

        /// @brief Pointer to the socket can driver.
        std::unique_ptr<drivers::socketcan::SocketCanReceiver> receiver_;

        std::unique_ptr<std::thread> receiver_thread_;
        
        // internal states
        /// @brief The name of the socket can interface.
        std::string interface_;

        /// @brief Socket can receive interval \f$[ns]\f$.
        std::chrono::nanoseconds interval_ns_;

        /// @brief Flag to determine whether to use can bus time.
        bool use_bus_time_;

        /// @brief Callback function to keep receiving can signals from another thread.
        void receive();
};

#endif // SOCKET_CAN_RECEIVER_NODE_HPP
