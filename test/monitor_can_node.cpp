// glibc include
#include <getopt.h>
#include <stdio.h>

// stl include
#include <functional>
#include <iostream>
#include <list>
#include <memory>
#include <string>

// ros2 include
#include <rclcpp/rclcpp.hpp>

// ros2 message include
#include <can_msgs/msg/frame.hpp>

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
      std::cout << "stamp: sec: " << std::dec << msg->header.stamp.sec
                << ", nanosec: " << msg->header.stamp.nanosec
                << "\nid: " << std::hex << msg->id
                << ", is_rtr: " << std::boolalpha << msg->is_error
                << ", is_extended: " << msg->is_extended
                << ", is_error: " << msg->is_error << ", dlc: " << std::dec
                << static_cast<int>(msg->dlc) << "\ndata: ";
      for (int i = 0; i < msg->dlc; i++) {
        std::cout << std::hex << static_cast<int>(msg->data[i]) << " ";
      }
      std::cout << "\n---" << std::endl;
    }
  }

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

  uint32_t can_id_filter_;
};

static const char *option = "d:h:";
struct option long_option[] = {{"dec", 0, NULL, 'd'}, {"hex", 1, NULL, 'h'}};

static const char *usage =
    "Usage: ros2 run nturt_can_parser monitor_can_node <OPTIONS>\n"
    "Options:\n"
    "    -d --dec    Set decimal filter\n"
    "    -h --hex    Set hexadecimal filter\n";

static void command_line_argument_error() {
  fprintf(stderr, "%s\n", usage);
  exit(1);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  rclcpp::NodeOptions options;

  uint32_t can_id_filter = 0;

  // prevent getopt to print error message to stderr
  opterr = 0;

  int opt = 0;
  while (1) {
    opt = getopt_long(argc, argv, option, long_option, NULL);

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
        printf("illegal option: %c\n", optopt);
        command_line_argument_error();
        break;
    }
  }

  if (argc > optind) {
    command_line_argument_error();
  }

  auto monitor_can_node = std::make_shared<MonitorCan>(options, can_id_filter);

  executor.add_node(monitor_can_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
