#include <cstdlib>

#include "zjslam/include/logging.hpp"
#include "zjslam/include/utils.hpp"
#include "zjslam/include/dataset/tum_vi.hpp"
#include "zjslam/include/camera/kannala_brandt.hpp"


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");

  Logger logger(argv);
  LOG(INFO) << "CXX standard: " << __cplusplus;

  std::cout.precision(6);
  cv::remap();
  cv::initUndistortRectifyMap();

  return 0;
}
