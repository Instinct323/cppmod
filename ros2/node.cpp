#include <rclcpp/rclcpp.hpp>


class NodeBase : public rclcpp::Node {

public:
    typedef std::shared_ptr<NodeBase> SharedPtr;

    // 构造方法
    NodeBase(const std::string &name) : rclcpp::Node(name) {}

    static SharedPtr create(const std::string &name = "demo") {
      auto node = std::make_shared<NodeBase>(name);
      RCLCPP_INFO(node->get_logger(), "Successful initialization.");
      return node;
    }
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(NodeBase::create());
  rclcpp::shutdown();
  return 0;
}
