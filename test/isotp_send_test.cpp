// std include
#include <functional>
#include <iostream>
#include <isotp.hpp>
#include <string>
#include <string.h>

// ros include
#include <ros/ros.h>

// ros interface include
#include <can_msgs/Frame.h>

class IsoTpSendTest {
    public:
        IsoTpSendTest(std::shared_ptr<ros::NodeHandle> _nh): nh_(_nh), can_pub_(_nh->advertise<can_msgs::Frame>("/sent_messages", 10)),
            can_sub_(_nh->subscribe("/received_messages", 10, &IsoTpSendTest::onCan, this)),
            isotp_timer_(_nh->createTimer(ros::Duration(1), &IsoTpSendTest::test_isotp_callback, this, true)),
            isotp(192, 512, 512,
                std::bind(&IsoTpSendTest::send_can, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                std::bind(&IsoTpSendTest::get_milli, this),
                std::bind(&IsoTpSendTest::error_out, this, std::placeholders::_1)) {

        }

        IsoTp isotp;
        std::shared_ptr<ros::NodeHandle> nh_;
        ros::Publisher can_pub_;
        ros::Subscriber can_sub_;
        ros::Timer isotp_timer_;

        int send_can(const uint32_t _id, const uint8_t *_data, const uint8_t _size) {
            can_msgs::Frame can_msg;
            can_msg.header.stamp = ros::Time::now();
            can_msg.is_extended = (_id > 0x11111111111);
            can_msg.id = _id;
            can_msg.dlc = _size;
            (void) memcpy(can_msg.data.c_array(), _data, _size);
            can_pub_.publish(can_msg);
            std::cout << "sending can data\n\tid: " << can_msg.id << ", size: " << std::to_string(can_msg.dlc) << "\n\tdata: ";
            for(int i = 0; i < _size; i++) {
                std::cout << std::to_string(can_msg.data[i]) << " ";
            }
            std::cout << std::endl;
            
            return ISOTP_RET_OK;
        }
        
        void onCan(const can_msgs::Frame::ConstPtr &_msg) {
            uint8_t data[8];
            for(int i = 0; i < _msg->dlc; i++) {
                data[i] = _msg->data[i];
            }
            isotp.on_can_message(data, _msg->dlc);
        }

        uint64_t get_milli() {
            return static_cast<uint64_t>(ros::Time::now().toSec() / 1000);
        }

        void error_out(const char *message) {
            ROS_ERROR(message);
        }

        void test_isotp_callback(const ros::TimerEvent&) {
            uint8_t data[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
            isotp.send(data, sizeof(data));
        }
};

int main(int argc, char **argv) {
    // register as a ros node
    ros::init(argc, argv, "isotp_send_test_node");

    // create a node handle
    auto nh = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
    
    IsoTpSendTest isotp_send_test(nh);

    // frequancy 1000 Hz
    ros::Rate loop_rate(1000);

    // main loop
    while (ros::ok()) {
        ros::spinOnce();
        isotp_send_test.isotp.update();
        loop_rate.sleep();
    }
    return 0;
}
