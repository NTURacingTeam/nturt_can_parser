#include "can_handler.hpp"

CanHandler::CanHandler(std::shared_ptr<ros::NodeHandle> _nh) : nh_(_nh),
    send_can_data_sub_(_nh->subscribe("/send_can_data", 100, &CanHandler::onSendCanData, this)),
    get_can_data_srv_(_nh->advertiseService("/get_can_data", &CanHandler::onGetCanData, this)) {

}

void CanHandler::onSendCanData(const nturt_can_parser::SendCanData &_msg) {

}

bool CanHandler::onGetCanData(nturt_can_parser::GetCanData::Request &_req,
    nturt_can_parser::GetCanData::Response &_res) {

    return true;
}
