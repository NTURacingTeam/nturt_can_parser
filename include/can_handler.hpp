/**
 * @file can_handler.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief ROS node wrapper for can parser.
 */

#ifndef CAN_HANDLER_HPP
#define CAN_HANDLER_HPP

// std include
#include <array>
#include <functional>
#include <memory>
#include <vector>

// ros include
#include <ros/ros.h>

// ros message include
#include <can_msgs/Frame.h>
#include <std_msgs/String.h>

// nturt include
#include "can_parser.hpp"
#include "nturt_ros_interface/GetCanData.h"
#include "nturt_ros_interface/RegisterCanNotification.h"
#include "nturt_ros_interface/UpdateCanData.h"


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

        /// @brief ROS subscriber to "/publish_can_frame".
        ros::Subscriber publish_frame_sub_;

        /// @brief ROS subscriber to "/update_can_data".
        ros::Subscriber update_data_sub_;

        /// @brief ROS service server to "/get_can_data".
        ros::ServiceServer get_data_srv_;

        /// @brief ROS service server to "/register_can_notification"
        ros::ServiceServer register_srv_;

        /// @brief CAN parser for parsing can data.
        CanParser can_parser_;

        /** @brief The cursed object.
         * 
         * The vector containing the pointer to the publisher to nodes that registered to be notified when can data update,
         * which is mapped by the name of the can data name that the nodes registed to, which iteslf is mapped by the frame
         * that contains these can data.
         */
        std::map<int, std::map<std::string, std::vector<std::shared_ptr<ros::Publisher>>>> registration_;

        /// @brief Callback function when receiving can message from "/received_messages".
        void onCan(const can_msgs::Frame::ConstPtr &_msg);

        /// @brief Callback function when receiving can message from "/update_can_data".
        void onUpdateCanData(const nturt_ros_interface::UpdateCanData::ConstPtr &_msg);

        /// @brief Callback function when receiving can data request service call from "/get_can_data".
        bool onGetCanData(nturt_ros_interface::GetCanData::Request &_req,
            nturt_ros_interface::GetCanData::Response &_res);

        /// @brief Callback function when receiving registration request service call form "/register_can_notification".
        bool onRegister(nturt_ros_interface::RegisterCanNotification::Request &_req,
            nturt_ros_interface::RegisterCanNotification::Response &_res);

        /**
         * @brief Function for publishing can signal.
         * @param _frame Can frame to be published.
         * @param _data The raw data of the can signal.
         */
        void publish(const FramePtr &_frame, const boost::array<uint8_t, 8> &_data);
};

#endif // CAN_HANDLER_HPP
