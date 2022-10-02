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
#define PAUSE printf("Press Enter key to continue..."); fgetc(stdin);

class CanHandlerTest {
    public:
        CanHandlerTest(std::shared_ptr<ros::NodeHandle> _nh) :
            can_pub_(_nh->advertise<can_msgs::Frame>("/received_messages", 10)),
            publish_frame_pub_(_nh->advertise<std_msgs::String>("/publish_can_frame", 10)),
            update_data_pub_(_nh->advertise<nturt_ros_interface::UpdateCanData>("/update_can_data", 10)),
            can_sub_(_nh->subscribe("/sent_messages", 10, &CanHandlerTest::onCan, this)),
            get_data_clt_(_nh->serviceClient<nturt_ros_interface::GetCanData>("/get_can_data")),
            register_clt_(_nh->serviceClient<nturt_ros_interface::RegisterCanNotification>("/register_can_notification")) {
        }
        
        void test_thread() {
            ROS_INFO("can handler test start.");
            PAUSE
            register_test();
        }

        void register_test() {
            // construct service call
            nturt_ros_interface::RegisterCanNotification srv;
            srv.request.node_name = ros::this_node::getName();
            // data name registering to be notified
            std::vector<std::string> data_name;
            data_name.push_back("control_board_temp");
            srv.request.data_name = data_name;
            
            // call service
            if(register_clt_.call(srv)) {
                ROS_INFO("registered topic name: %s", srv.response.topic.c_str());
            }
            else {
                ROS_ERROR("register failed");
            }
        }
    private:
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

        void onCan(const can_msgs::Frame::ConstPtr &_msg) const {
            ROS_INFO("Received can frame with id: %d", _msg->id);
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
