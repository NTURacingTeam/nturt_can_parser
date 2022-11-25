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

class IsoTpReceiveTest {
    public:
        IsoTpReceiveTest(std::shared_ptr<ros::NodeHandle> _nh): nh_(_nh), can_pub_(_nh->advertise<can_msgs::Frame>("/received_messages", 10)),
            can_sub_(_nh->subscribe("/sent_messages", 10, &IsoTpReceiveTest::onCan, this)),
            isotp(192, 512, 512,
                std::bind(&IsoTpReceiveTest::send_can, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                std::bind(&IsoTpReceiveTest::get_milli, this),
                std::bind(&IsoTpReceiveTest::error_out, this, std::placeholders::_1)) {

        }

        IsoTp isotp;
        std::shared_ptr<ros::NodeHandle> nh_;
        ros::Publisher can_pub_;
        ros::Subscriber can_sub_;
        
        void onCan(const can_msgs::Frame::ConstPtr &_msg) {
            uint8_t data[8];
            for(int i = 0; i < _msg->dlc; i++) {
                data[i] = _msg->data[i];
            }
            isotp.on_can_message(data, _msg->dlc);
        }

        int send_can(const uint32_t _id, const uint8_t *_data, const uint8_t _size) {
            can_msgs::Frame can_msg;
            can_msg.header.stamp = ros::Time::now();
            can_msg.is_extended = (_id > 0x11111111111);
            can_msg.id = _id;
            can_msg.dlc = _size;
            (void) memcpy(can_msg.data.c_array(), _data, _size);
            can_pub_.publish(can_msg);            
            return ISOTP_RET_OK;
        }

        uint64_t get_milli() {
            return static_cast<uint64_t>(ros::Time::now().toSec() / 1000);
        }

        void error_out(const char *message) {
            ROS_ERROR(message);
        }
};

int main(int argc, char **argv) {
    // register as a ros node
    ros::init(argc, argv, "isotp_receive_test_node");

    // create a node handle
    auto nh = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
    
    IsoTpReceiveTest isotp_receive_test(nh);

    uint8_t data[4095];
    uint16_t out_size;

    // frequancy 1000 Hz
    ros::Rate loop_rate(1000);

    // main loop
    while (ros::ok()) {
        ros::spinOnce();
        isotp_receive_test.isotp.update();
        if(isotp_receive_test.isotp.receive(data, sizeof(data), out_size) == ISOTP_RET_OK) {
            std::cout << "received data\n\tsize: " << out_size << "\n\tdata: ";
            for(int i = 0; i < out_size; i++) {
                std::cout << std::to_string(data[i]) << " ";
            }
            std::cout << std::endl;
        }
        loop_rate.sleep();
    }
    return 0;
}
