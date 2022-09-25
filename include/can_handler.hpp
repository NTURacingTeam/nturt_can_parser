/**
 * @file can_handler.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief ROS node wrapper for can parser.
 */

#ifndef CAN_HANDLER_HPP
#define CAN_HANDLER_HPP

// std include
#include <memory>
#include <vector>

// ros include
#include <ros/ros.h>

// ros message include
#include <can_msgs/Frame.h>

// nturt include
#include "can_parser.hpp"
#include "nturt_ros_interface/GetCanData.h"
#include "nturt_ros_interface/RegisterCanNotification.h"
#include "nturt_ros_interface/SendCanData.h"

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief 
 */
struct Registration {
    Registration();

    /// @brief Publisher to corresponding registered node.
    ros::Publisher notify_pub_;

    /// @brief Data list when updated to publish to the registered node.
    std::vector<std::string> data_list_;
};

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for ros wrapper for can parser.
 */
class CanHandler {
    public:
        CanHandler(std::shared_ptr<ros::NodeHandle> _nh);

    private:
        /// @brief ROS node handler.
        std::shared_ptr<ros::NodeHandle> nh_;

        /// @brief ROS publisher to "/sent_messages".
        ros::Publisher can_pub_;

        /// @brief ROS sbscriber to "/received_messages".
        ros::Subscriber can_sub_;

        /// @brief ROS subscriber to "/send_can_data".
        ros::Subscriber send_data_sub_;

        /// @brief ROS service server to "/get_can_data".
        ros::ServiceServer get_data_srv_;

        /// @brief ROS service server to "/register_can_notification"
        ros::ServiceServer register_srv_;

        /// @brief CAN parser for parsing can data.
        CanParser can_parser_;

        /**
         * @brief Callback function when receiving can message from "/received_messages".
         * @param _msg 
         */
        void onCan(const can_msgs::Frame::ConstPtr &_msg);

        /**
         * @brief Callback function when receiving can message from "/send_can_data".
         * @param _msg 
         * @return true
         */
        void onSendCanData(const nturt_ros_interface::SendCanData::ConstPtr &_msg);

        /**
         * @brief Callback function when receiving can data request service call from "/get_can_data".
         * @param _req 
         * @param _res 
         * @return true 
         */
        bool onGetCanData(nturt_ros_interface::GetCanData::Request &_req,
            nturt_ros_interface::GetCanData::Response &_res);

        /**
         * @brief Callback function when receiving registration request service call form "/register_can_notification".
         * @param _req 
         * @param _res 
         * @return true 
         */
        bool onRegister(nturt_ros_interface::RegisterCanNotification::Request &_req,
            nturt_ros_interface::RegisterCanNotification::Response &_res);
};

#endif // CAN_HANDLER_HPP
