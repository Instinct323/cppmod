#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include "utils.h"

#define ROOT "../data"


class NodeBase : public rclcpp::Node {
public:
    typedef std::shared_ptr<NodeBase> SharedPtr;

    // 构造方法
    static SharedPtr create(const std::string &name = "demo") {
      auto node = std::make_shared<NodeBase>(name);
      RCLCPP_INFO(node->get_logger(), "Successful initialization.");
      return node;
    }

    NodeBase(const std::string &name) : rclcpp::Node(name) {}
};


int main(int argc, char **argv) {
  setenv("DISPLAY", "host.docker.internal:0", 1);
  Logger logger(argv);

  // test
  rclcpp::init(argc, argv);
  auto node = NodeBase::create();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
