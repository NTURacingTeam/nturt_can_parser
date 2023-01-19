// glibc include
#include <pthread.h>
#include <sched.h>
#include <sys/types.h>
#include <unistd.h>

// std include
#include <list>
#include <memory>

// ros2 include
#include "rclcpp/rate.hpp"

// nturt include
#include "nturt_can_parser/socket_can_receiver.hpp"
#include "nturt_realtime_utils/memory_lock.hpp"
#include "nturt_realtime_utils/scheduling.hpp"

static const std::list<std::string> realtime_keys = {
    "realtime",
    "real-time",
    "real_time",
    "--realtime",
    "--real-time",
    "--real_time",
};

int main(int argc, char **argv) {
    // real-time configuration
    if (argc > 1) {
        for (auto & realtime_key : realtime_keys) {
            if (std::string(argv[1]) == realtime_key) {
                lock_memory();
                set_thread_scheduling(pthread_self(), SCHED_FIFO, 80);
                break;
            }
        }
    }

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::NodeOptions options;

    auto socket_can_sender_node = std::make_shared<SocketCanReceiver>(options);

    executor.add_node(socket_can_sender_node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
