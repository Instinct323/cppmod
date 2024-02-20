#include <rclcpp/rclcpp.hpp>

#include "utils.h"

#define ROOT "../data"


int main(int argc, char **argv) {
  setenv("DISPLAY", "host.docker.internal:0", 1);
  Logger logger(argv);

  // test
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("node");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
