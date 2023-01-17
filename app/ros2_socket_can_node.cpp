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
#include "nturt_realtime_utils/memory_lock.hpp"
#include "nturt_realtime_utils/scheduling.hpp"

int main(int argc, char **argv) {
    // real-time configuration
    lock_memory();
    set_thread_scheduling(pthread_self(), SCHED_FIFO, 80);

    rclcpp::init(argc, argv);

    rclcpp::executors::StaticSingleThreadedExecutor executor;
    rclcpp::NodeOptions options;

    rclcpp::Node::SharedPtr ros2_socket_can_node = std::make_shared<drivers::socketcan::SocketCanReceiverNode>(options);

    executor.add_node(ros2_socket_can_node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
