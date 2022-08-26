// std include
#include <memory>

// ros include
#include <ros/ros.h>

// ros message include
#include "can_msgs/Frame.h"


// can parser include
#include "can_parser.hpp"

int main(int argc, char **argv) {
    // register as a ros node
    ros::init(argc, argv, "nturt_can_parser_node");

    // create a node handle
    std::shared_ptr<ros::NodeHandle> nh = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());

    // frequancy 1000 Hz
    ros::Rate loop_rate(1000);

    CanParser test;
    // Main loop
    while (ros::ok()) {
        ros::spinOnce();
        //std::cout << "while\n";
        loop_rate.sleep();
    }
    return 0;
}
