// glibc include
#include <getopt.h>
#include <stdio.h>

// stl include
#include <functional>
#include <list>
#include <memory>
#include <string>

// ros2 include
#include <rclcpp/rclcpp.hpp>

// ros2 message include
#include <can_msgs/msg/frame.hpp>

#define COLOR_REST "\033[0m"
#define HIGHLIGHT "\033[0;1m"
#define COLOR_RED "\033[1;31m"

class MonitorCan : public rclcpp::Node {
 public:
  /// @brief Constructor of MonitorCan.
  MonitorCan(rclcpp::NodeOptions options, uint32_t can_id_filter = 0)
      : Node("screen_test_node", options),
        can_sub_(this->create_subscription<can_msgs::msg::Frame>(
            "/from_can_bus", 50,
            std::bind(&MonitorCan::onCan, this, std::placeholders::_1))),
        can_id_filter_(can_id_filter) {}

 private:
  void onCan(const std::shared_ptr<can_msgs::msg::Frame> msg) {
    if (can_id_filter_ == 0 || msg->id == can_id_filter_) {
      printf(
          "stamp: sec: %d, nanosec: %d\nid: 0x%X, is_rtr: %s, is_extended: "
          "%s, is_error: %s, dlc: %d\ndata: ",
          msg->header.stamp.sec, msg->header.stamp.nanosec, msg->id,
          msg->is_rtr ? "true" : "false", msg->is_extended ? "true" : "false",
          msg->is_error ? "true" : "false", msg->dlc);

      for (int i = 0; i < msg->dlc; i++) {
        printf("0x%X ", msg->data[i]);
      }
      std::cout << "\n---" << std::endl;
    }
  }

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

  uint32_t can_id_filter_;
};

static const char *option = "d:h:";
struct option long_option[] = {
    {"dec", 0, NULL, 'd'},
    {"hex", 1, NULL, 'h'},
    {0, 0, 0, 0},
};

static const char *usage =
    "Monitor can bus message.\n"
    "\n"
    "Usage: ros2 run nturt_can_parser monitor_can_node <OPTIONS>\n"
    "Options:\n"
    "    -d --dec    Set decimal filter\n"
    "    -h --hex    Set hexadecimal filter\n";

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  rclcpp::NodeOptions options;

  uint32_t can_id_filter = 0;

  // prevent getopt to print error message to stderr
  opterr = 0;

  while (1) {
    int opt = getopt_long(argc, argv, option, long_option, NULL);

    if (opt == -1) {
      break;
    }

    switch (opt) {
      case 'd':
        can_id_filter = std::stoul(optarg, nullptr, 10);
        break;

      case 'h':
        can_id_filter = std::stoul(optarg, nullptr, 16);
        break;

      case '?':
        fprintf(stderr,
                COLOR_RED "Error:" HIGHLIGHT " Unknown option: %c\n" COLOR_REST,
                optopt);
        printf("%s\n", usage);
        exit(1);
    }
  }

  if (argc > optind) {
    fprintf(stderr,
            COLOR_RED "Error:" HIGHLIGHT
                      " Argument given, expect no argument %c\n" COLOR_REST,
            optopt);
    printf("%s\n", usage);
    exit(1);
  }

  auto monitor_can_node = std::make_shared<MonitorCan>(options, can_id_filter);

  executor.add_node(monitor_can_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
