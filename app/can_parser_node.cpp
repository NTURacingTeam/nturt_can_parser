// std include
#include <memory>
#include <string>

// ros include
#include <ros/ros.h>

// nturt include
#include "can_handler.hpp"

int main(int argc, char **argv) {
    // register as a ros node
    ros::init(argc, argv, "nturt_can_parser_node");

    // create a node handle
    auto nh = std::make_shared<ros::NodeHandle>(ros::NodeHandle());

    // frequancy 1000 Hz
    ros::Rate loop_rate(1000);

    CanHandler can_handler(nh);

    // main loop
    while (ros::ok()) {
        ros::spinOnce();
        can_handler.update();
        loop_rate.sleep();
    }
    return 0;
}
