#include <cstdlib>

#include "utils.hpp"
#include "zjslam/include/utils.hpp"


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");

  Logger logger(argv);
  LOG(INFO) << "CXX standard: " << __cplusplus;

  std::cout.precision(6);

  ImageLoader imgLoader;
  cv::Mat img1 = imgLoader("/home/workbench/data/both.png");
  cv::imshow("img1", img1);
  cv::waitKey(0);

  return 0;
}
