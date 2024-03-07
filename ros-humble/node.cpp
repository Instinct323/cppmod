#include <rclcpp/rclcpp.hpp>


class Node : public rclcpp::Node {

public:
    typedef std::shared_ptr<Node> SharedPtr;

    Node(const std::string &name) : rclcpp::Node(name) {}
};


// ros2 pkg create zjros2 --dependencies rclcpp
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Node>("demo");
  RCLCPP_INFO(node->get_logger(), "Successful initialization.");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
