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
    //std::vector<std::pair<std::string, std::string>> result = test.get_key(123);
    //std::cout << result[0].first << "\n" << result[0].second << "\n";
    // Main loop
    auto result = test.get_key(0x080AD092);
    for (auto it = result.begin(); it != result.end(); it++) {
        cout << it->first << "\n";
        cout << it->second << "\n";
    }
    while (ros::ok()) {
        ros::spinOnce();
        //std::cout << "while\n";
        loop_rate.sleep();
    }
    return 0;
}
