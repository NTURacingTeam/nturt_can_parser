#include "can_handler.hpp"

CanHandler::CanHandler(std::shared_ptr<ros::NodeHandle> _nh) : nh_(_nh),
    can_pub_(_nh->advertise<can_msgs::Frame>("/sent_messages", 10)),
    can_sub_(_nh->subscribe("received_messages", 10, &CanHandler::onCan, this)),
    publish_frame_sub_(_nh->),
    update_data_sub_(_nh->subscribe("/update_can_data", 10, &CanHandler::onUpdateCanData, this)),
    get_data_srv_(_nh->advertiseService("/get_can_data", &CanHandler::onGetCanData, this)),
    register_srv_(_nh->advertiseService("/register_can_notification", &CanHandler::onRegister, this)) {

    // asume that can parser had already been initialized
    // initialize the map key in registration_
    IdFrameset frameset = can_parser_.get_id_frameset();
    for(auto frame_it = frameset.begin(); frame_it != frameset.end(); frame_it++) {
        for(auto data_it = frame_it->second->dataset_.begin(); data_it != frame_it->second->dataset_.end(); data_it++) {
            registration_[frame_it->first][data_it->first];
        }
    }
}

void CanHandler::onCan(const can_msgs::Frame::ConstPtr &_msg) {
    // update the frame stored in the can parser
    FramePtr updated_frame = can_parser_.update_frame(_msg->id, _msg->data);

    // publish to registered nodes
    for(auto data_it = registration_[updated_frame->id_].begin(); data_it != registration_[updated_frame->id_].end(); data_it++) {
        // message to update can data to registered node
        nturt_ros_interface::UpdateCanData data_msg;
        data_msg.name = data_it->first;
        data_msg.name = updated_frame->dataset_[data_it->first]->last_data_;

        for(auto pub_it = data_it->second.begin(); pub_it != data_it->second.end(); pub_it++) {
            (*pub_it)->publish(data_msg);
        }
    }
}

void CanHandler::onUpdateCanData(const nturt_ros_interface::UpdateCanData::ConstPtr &_msg) {

}

bool CanHandler::onGetCanData(nturt_ros_interface::GetCanData::Request &_req,
    nturt_ros_interface::GetCanData::Response &_res) {

    DataPtr data = can_parser_.get_data(_req.name);

    // if data is null_ptr
    if(!data) {
        ROS_ERROR("Error: Data: %s not found.", _req.name.c_str());
        return false;
    }

    _res.data = data->last_data_;
    return true;
}

bool CanHandler::onRegister(nturt_ros_interface::RegisterCanNotification::Request &_req,
    nturt_ros_interface::RegisterCanNotification::Response &_res) {
        
    for(auto it = _req.data_name.begin(); it != _req.data_name.end(); it++) {
        
    }
    return true;
}

void CanHandler::publish(const FramePtr &_frame, const boost::array<uint8_t, 8> &_data) {
    // if _frame is null_ptr
    if(!_frame) {
        ROS_ERROR("Error: Frame not fround.");
        return;
    }

    // construct can frame
    can_msgs::Frame can_msg;
    can_msg.header.stamp = ros::Time::now();
    can_msg.is_extended = _frame->is_extended_id_;
    can_msg.id = _frame->id_;
    can_msg.data = _data;
    
    // publish
    can_pub_.publish(can_msg);
}
