#include "can_handler.hpp"

CanHandler::CanHandler(std::shared_ptr<ros::NodeHandle> _nh) : nh_(_nh),
    can_pub_(_nh->advertise<can_msgs::Frame>("/sent_messages", 10)),
    can_sub_(_nh->subscribe("/received_messages", 10, &CanHandler::onCan, this)),
    publish_frame_sub_(_nh->subscribe("/publish_can_frame", 10, &CanHandler::onPublishCanFrame, this)),
    update_data_sub_(_nh->subscribe("/update_can_data", 10, &CanHandler::onUpdateCanData, this)),
    get_data_srv_(_nh->advertiseService("/get_can_data", &CanHandler::onGetCanData, this)) {
    
    // get can yaml file
    std::string can_config;
    ros::param::get("~can_config", can_config);

    // init can parser
    can_parser_.init(can_config);

    // initialize the map key in registration_
    IdFrameset frameset = can_parser_.get_id_frameset();
    for(auto frame_it = frameset.begin(); frame_it != frameset.end(); frame_it++) {
        for(auto data_it = frame_it->second->dataset_.begin(); data_it != frame_it->second->dataset_.end(); data_it++) {
            registration_[frame_it->first][data_it->first];
        }
    }
    
    // advertise "/register_can_notification" service until finishing initializin the map key in registration_
    register_srv_ = nh_->advertiseService("/register_can_notification", &CanHandler::onRegister, this);

    // initiate last_time_
    last_time_ = ros::Time::now();
}

void CanHandler::update() {
    ros::Time current_time = ros::Time::now();

    // periodically publish can frame
    can_parser_.periodic_publish((current_time - last_time_).toSec(),
        std::bind(&CanHandler::publish, this, std::placeholders::_1, std::placeholders::_2));
    
    last_time_ = current_time;
}

void CanHandler::onCan(const can_msgs::Frame::ConstPtr &_msg) {
    // update the frame stored in can parser
    FramePtr frame = can_parser_.update_frame(_msg->id, _msg->data);

    // if frame is nullptr
    if(!frame) {
        ROS_ERROR("Error: Frame not found when receiving can signal id: \"%d\"", _msg->id);
        return;
    }

    // publish to registered nodes
    for(auto data_it = registration_[frame->id_].begin(); data_it != registration_[frame->id_].end(); data_it++) {
        // message to update can data to registered node
        nturt_ros_interface::UpdateCanData data_msg;
        data_msg.name = data_it->first;
        data_msg.data = frame->dataset_[data_it->first]->last_data_;

        for(auto pub_it = data_it->second.begin(); pub_it != data_it->second.end(); pub_it++) {
            (*pub_it)->publish(data_msg);
        }
    }
}

void CanHandler::onPublishCanFrame(const std_msgs::String::ConstPtr &_msg) {
    // if frame publish is unsuccessfully
    if(!can_parser_.publish(_msg->data, std::bind(&CanHandler::publish, this, std::placeholders::_1, std::placeholders::_2))) {
        ROS_ERROR("Error: Frame not found when publishing can frame: \"%s\".", _msg->data.c_str());
    }
}

void CanHandler::onUpdateCanData(const nturt_ros_interface::UpdateCanData::ConstPtr &_msg) {
    // if data not found
    if(!can_parser_.update_data(_msg->name, _msg->data)) {
        ROS_ERROR("Error: Data not found when updating can data: \"%s\"", _msg->name.c_str());
    }
}

bool CanHandler::onGetCanData(nturt_ros_interface::GetCanData::Request &_req,
    nturt_ros_interface::GetCanData::Response &_res) {
    
    DataPtr data = can_parser_.get_data(_req.name);

    // if data is nullptr
    if(!data) {
        ROS_ERROR("Error: Data not found when getting can data: \"%s\".", _req.name.c_str());
        return false;
    }

    _res.data = data->last_data_;
    return true;
}

bool CanHandler::onRegister(nturt_ros_interface::RegisterCanNotification::Request &_req,
    nturt_ros_interface::RegisterCanNotification::Response &_res) {
    
    Dataset dataset = can_parser_.get_dataset();

    // check if all register data exist
    for(std::string &data : _req.data_name) {
        if(dataset.find(data) == dataset.end()) {
            ROS_ERROR("Error: Data not found when setting registration can data: \"%s\" from node: \"%s\"", data.c_str(), _req.node_name.c_str());
            return false;
        }
    }
    
    // topic for the notification
    _res.topic = "/can_notification" + _req.node_name;
    auto publisher = std::make_shared<ros::Publisher>(nh_->advertise<nturt_ros_interface::UpdateCanData>(_res.topic, 10));

    // register
    for(std::string &data : _req.data_name) {
        registration_[dataset[data]->frame_->id_][data].push_back(publisher);
    }

    return true;
}

void CanHandler::publish(const FramePtr &_frame, const boost::array<uint8_t, 8> &_data) {
    // construct can frame
    can_msgs::Frame can_msg;
    can_msg.header.stamp = ros::Time::now();
    can_msg.is_extended = _frame->is_extended_id_;
    can_msg.id = _frame->id_;
    can_msg.data = _data;
    
    // publish
    can_pub_.publish(can_msg);
}
