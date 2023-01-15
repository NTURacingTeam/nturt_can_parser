// glibc include
#include <pthread.h>
#include <sched.h>
#include <sys/types.h>
#include <unistd.h>

// std include
#include <memory>

// ros2 include
#include "rclcpp/rate.hpp"

// nturt include
#include "nturt_can_parser/socket_can_sender_node.hpp"
#include "nturt_can_parser/socket_can_receiver_node.hpp"

int main(int argc, char **argv) {
    struct sched_param param;
    param.sched_priority = 80;
    sched_setscheduler(getpid(), SCHED_FIFO, &param);

    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    rclcpp::NodeOptions options;
    executor.add_node(std::make_shared<drivers::socketcan::SocketCanReceiverNode>(options));
    executor.spin();

    return 0;
}
