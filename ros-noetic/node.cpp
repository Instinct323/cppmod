#include <ros/ros.h>


class Node : public ros::NodeHandle {

public:
    Node() : ros::NodeHandle() {}
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "demo");
  Node node;
  ros::spin();
  return 0;
}
