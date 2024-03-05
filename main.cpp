#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "utils.h"

#define ROOT "../data"


int main(int argc, char **argv) {
  setenv("DISPLAY", "host.docker.internal:0", 1);
  Logger logger(argv);

  return 0;
}
