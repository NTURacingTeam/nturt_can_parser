#include "can_handler.hpp"

CanHandler::CanHandler(std::shared_ptr<ros::NodeHandle> _nh) : nh_(_nh),
    can_pub_(_nh->advertise<can_msgs::Frame>("/sent_messages", 10)),
    can_sub_(_nh->subscribe("received_messages", 10, &CanHandler::onCan, this)),
    send_data_sub_(_nh->subscribe("/send_can_data", 10, &CanHandler::onSendCanData, this)),
    get_data_srv_(_nh->advertiseService("/get_can_data", &CanHandler::onGetCanData, this)),
    register_srv_(_nh->advertiseService("/register_can_notification", &CanHandler::onRegister, this)) {

}

void CanHandler::onCan(const can_msgs::Frame::ConstPtr &_msg) {

}

void CanHandler::onSendCanData(const nturt_ros_interface::SendCanData::ConstPtr &_msg) {

}

bool CanHandler::onGetCanData(nturt_ros_interface::GetCanData::Request &_req,
    nturt_ros_interface::GetCanData::Response &_res) {

    return true;
}

bool CanHandler::onRegister(nturt_ros_interface::RegisterCanNotification::Request &_req,
    nturt_ros_interface::RegisterCanNotification::Response &_res) {
        
    
    return true;
}
