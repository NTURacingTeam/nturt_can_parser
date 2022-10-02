// std include
#include <memory>
#include <string>
#include <thread>
#include <vector>

// ros include
#include <can_msgs/Frame.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

// nturt include
#include "can_handler.hpp"
#include "nturt_ros_interface/GetCanData.h"
#include "nturt_ros_interface/RegisterCanNotification.h"
#include "nturt_ros_interface/UpdateCanData.h"

// system("pause") for linux
#define PAUSE ROS_INFO("Press Enter key to continue..."); fgetc(stdin);

class CanHandlerTest {
    public:
        CanHandlerTest(std::shared_ptr<ros::NodeHandle> _nh) :
            nh_(_nh),
            can_pub_(_nh->advertise<can_msgs::Frame>("/received_messages", 10)),
            publish_frame_pub_(_nh->advertise<std_msgs::String>("/publish_can_frame", 10)),
            update_data_pub_(_nh->advertise<nturt_ros_interface::UpdateCanData>("/update_can_data", 10)),
            can_sub_(_nh->subscribe("/sent_messages", 10, &CanHandlerTest::onCan, this)),
            get_data_clt_(_nh->serviceClient<nturt_ros_interface::GetCanData>("/get_can_data")),
            register_clt_(_nh->serviceClient<nturt_ros_interface::RegisterCanNotification>("/register_can_notification")) {
        }
        
        // thread for testing
        void test_thread() {
            ROS_INFO("can handler test start");
            PAUSE
            register_test();
            PAUSE
            update_test();
            PAUSE
            get_data_test();
            PAUSE
            publish_test();
            ROS_INFO("can handler test end");
        }

    private:
        std::shared_ptr<ros::NodeHandle> nh_;
        // publishers
        ros::Publisher can_pub_;
        ros::Publisher publish_frame_pub_;
        ros::Publisher update_data_pub_;

        // subscribers
        ros::Subscriber can_sub_;
        ros::Subscriber register_data_sub_;

        // service clients
        ros::ServiceClient get_data_clt_;
        ros::ServiceClient register_clt_;

        bool show_can_ = false;

        bool show_register_ = false;

        // callback function when receiving can message from "/sent_messages"
        void onCan(const can_msgs::Frame::ConstPtr &_msg) const {
            if(show_can_) {
                ROS_INFO("received can frame with id: \"%d\"", _msg->id);
            }
        }

        // callback function when receiving register message.
        void onRegister(const nturt_ros_interface::UpdateCanData::ConstPtr _msg) {
            if(show_register_) {
                ROS_INFO("received register message with name: \"%s\" and data: \"%f\"", _msg->name.c_str(), _msg->data);
            }
        }

        // test for register notification
        void register_test() {
            ROS_INFO("register test begin");
            show_register_ = true;

            // register service call
            nturt_ros_interface::RegisterCanNotification register_srv;
            register_srv.request.node_name = ros::this_node::getName();
            // data name registering to be notified
            std::vector<std::string> data_name;
            data_name.push_back("control_board_temp");
            register_srv.request.data_name = data_name;
            
            // call service
            if(register_clt_.call(register_srv)) {
                ROS_INFO("successfully registered topic name: \"%s\"", register_srv.response.topic.c_str());
            }
            else {
                ROS_ERROR("register failed");
            }

            // subscribe to the register topic
            register_data_sub_ = nh_->subscribe(register_srv.response.topic, 10, &CanHandlerTest::onRegister, this);

            ROS_INFO("send test to registered topic, expect register callback trigger");

            PAUSE

            // fake can message for testing
            can_msgs::Frame frame_msg;
            frame_msg.header.stamp = ros::Time::now();
            frame_msg.id = 0x0A1;
            frame_msg.is_extended = false;
            frame_msg.dlc = 8;
            frame_msg.data = {1, 1, 0, 0, 0, 0, 0, 0};
            can_pub_.publish(frame_msg);

            ros::Duration(0.1).sleep();

            show_register_ = false;
            ROS_INFO("register test end");
        }
        
        // test for update can data
        void update_test() {
            ROS_INFO("update test begin");
            show_can_ = true;

            ROS_INFO("send test to change can data output, expect can data to be different");
            PAUSE

            // fake can update for testing
            nturt_ros_interface::UpdateCanData update_msg;
            update_msg.name = "torque_command";
            update_msg.data = 1000;
            update_data_pub_.publish(update_msg);

            ros::Duration(2.0).sleep();

            show_can_ = false;
            ROS_INFO("update test end");
        }

        void get_data_test() {
            ROS_INFO("get data test begin");

            // get data service call
            nturt_ros_interface::GetCanData get_srv;
            get_srv.request.name = "control_board_temp";

            // call service
            if(get_data_clt_.call(get_srv)) {
                ROS_INFO("successfully get data \"control_board_temp\" with value: \"%f\"", get_srv.response.data);
            }
            else {
                ROS_ERROR("get data failed");
            }

            ROS_INFO("get data test end");
        }

        void publish_test() {
            ROS_INFO("publish test begin");
            show_can_ = true;

            std_msgs::String publish_msg;
            publish_msg.data = "mcu_board";
            publish_frame_pub_.publish(publish_msg);
            
            ros::Duration(0.1).sleep();

            show_can_ = false;
            ROS_INFO("publish test end");
        }
};

int main(int argc, char **argv) {
    // register as a ros node
    ros::init(argc, argv, "nturt_can_handler_test");

    // create a node handle
    auto nh = std::make_shared<ros::NodeHandle>(ros::NodeHandle());

    CanHandlerTest can_handler_test(nh);

    std::thread test_thread(std::bind(&CanHandlerTest::test_thread, can_handler_test));

    ros::spin();
    return 0;
}
