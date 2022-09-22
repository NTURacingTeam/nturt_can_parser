/**
 * @file can_handler.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief ROS node wrapper for can parser.
 */

#ifndef CAN_HANDLER_HPP
#define CAN_HANDLER_HPP

// std include
#include <memory>

// ros include
#include <ros/ros.h>

// ros message include
#include <can_msgs/Frame.h>

// nturt include
#include "can_parser.hpp"
#include "nturt_can_parser/SendCanData.h"
#include "nturt_can_parser/GetCanData.h"

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
        ros::Subscriber send_can_data_sub_;
        
        /// @brief ROS service server to "/get_can_data".
        ros::ServiceServer get_can_data_srv_;

        /// @brief CAN parser for parsing can data.
        CanParser can_parser_;

        void onCan(const can_msgs::Frame::ConstPtr &_msg);

        void onSendCanData(const nturt_can_parser::SendCanData::ConstPtr &_msg);

        bool onGetCanData(nturt_can_parser::GetCanData::Request &_req,
            nturt_can_parser::GetCanData::Response &_res);
};

#endif // CAN_HANDLER_HPP
