// std include
#include <math.h>
#include <functional>
#include <iostream>
#include <isotp.hpp>
#include <string>
#include <string.h>

// ros include
#include <ros/ros.h>

// ros interface include
#include <can_msgs/Frame.h>

class FakeCanSignalGenerator {
    public:
        FakeCanSignalGenerator(std::shared_ptr<ros::NodeHandle> _nh): nh_(_nh), can_pub_(_nh->advertise<can_msgs::Frame>("/received_messages", 10)),
            fast_frame_timer_(_nh->createTimer(ros::Duration(0.01), &FakeCanSignalGenerator::fast_frame_callback, this, false)),
            slow_frame_timer_(_nh->createTimer(ros::Duration(0.1), &FakeCanSignalGenerator::slow_frame_callback, this, false)) {

        }

    private:
        std::shared_ptr<ros::NodeHandle> nh_;

        ros::Publisher can_pub_;

        ros::Timer fast_frame_timer_;

        ros::Timer slow_frame_timer_;

        void fast_frame_callback (const ros::TimerEvent&) {
            double now = ros::Time::now().toSec(), intpart;
            uint8_t now256 = 256 * std::modf(now / 10, &intpart); // 0 ~ 255 of period 10s
            uint8_t now2 = static_cast<int>(now) % 2; // 0 ~ 1 of period 2s

            can_msgs::Frame can_msg;
            can_msg.header.stamp = ros::Time::now();
         
            // mcu_motor_position
            can_msg.id = 0x0A5;
            can_msg.is_extended = false;
            can_msg.dlc = 8;
            can_msg.data = {0, 0, now256, 0, 0, 0, 0, 0};
            can_pub_.publish(can_msg);

            // mcu_voltage
            can_msg.id = 0x0A7;
            can_msg.is_extended = false;
            can_msg.dlc = 8;
            can_msg.data = {now256, 0, 0, 0, 0, 0, 0, 0};
            can_pub_.publish(can_msg);

            // imu_acceleration
            can_msg.id = 0x188;
            can_msg.is_extended = false;
            can_msg.dlc = 6;
            can_msg.data = {now256, 0, now256, 0, now256, 0, 0, 0};
            can_pub_.publish(can_msg);

            // imu_quternion
            can_msg.id = 0x488;
            can_msg.is_extended = false;
            can_msg.dlc = 8;
            can_msg.data = {now256, 0, now256, 0, now256, 0, now256, 0};
            can_pub_.publish(can_msg);

            // imu_gyro
            can_msg.id = 0x288;
            can_msg.is_extended = false;
            can_msg.dlc = 6;
            can_msg.data = {now256, 0, now256, 0, now256, 0, 0, 0};
            can_pub_.publish(can_msg);

            // front_box_2
            can_msg.id = 0x080AD092;
            can_msg.is_extended = true;
            can_msg.dlc = 8;
            can_msg.data = {now256, now256, now256, now256, now256, now256, now256, 2 * now2 + now2};
            can_pub_.publish(can_msg);

        }

        void slow_frame_callback (const ros::TimerEvent&) {
            double now = ros::Time::now().toSec(), intpart;
            uint8_t now256 = 256 * std::modf(now / 10, &intpart); // 0 ~ 255 of period 10s
            uint8_t now2 = static_cast<int>(now) % 2; // 0 ~ 1 of period 2s

            can_msgs::Frame can_msg;
            can_msg.header.stamp = ros::Time::now();

            // mcu_temperature_2
            can_msg.id = 0x0A1;
            can_msg.is_extended = false;
            can_msg.dlc = 8;
            can_msg.data = {now256, 0, 0, 0, 0, 0, 0, 0};
            can_pub_.publish(can_msg);

            // mcu_temperature_3
            can_msg.id = 0x0A2;
            can_msg.is_extended = false;
            can_msg.dlc = 8;
            can_msg.data = {0, 0, 0, 0, now256, 0, 0, 0};
            can_pub_.publish(can_msg);

            // front_box_1
            can_msg.id = 0x080AD091;
            can_msg.is_extended = true;
            can_msg.dlc = 8;
            can_msg.data = {0, now256, 0, now256, now256, now256, now256, now256};
            can_pub_.publish(can_msg);

            // rear_box_1
            can_msg.id = 0x080AD093;
            can_msg.is_extended = true;
            can_msg.dlc = 8;
            can_msg.data = {0, now256, 0, now256, now256, now256, now256, now256};
            can_pub_.publish(can_msg);

            // rear_box_2
            can_msg.id = 0x080AD094;
            can_msg.is_extended = true;
            can_msg.dlc = 8;
            can_msg.data = {0, now256, 0, now256, 0, 0, 0, 0};
            can_pub_.publish(can_msg);

            // dashboard
            can_msg.id = 0x080AD095;
            can_msg.is_extended = true;
            can_msg.dlc = 8;
            can_msg.data = {now2, now2, 0, 0, 0, 0, 0, now2};
            can_pub_.publish(can_msg);
        }
};

int main(int argc, char **argv) {
    // register as a ros node
    ros::init(argc, argv, "fake_can_sgnal_generator_node");

    // create a node handle
    auto nh = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
    

    FakeCanSignalGenerator fake_can_sgnal_generator(nh);

    // frequancy 1000 Hz
    ros::Rate loop_rate(1000);

    // main loop
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
