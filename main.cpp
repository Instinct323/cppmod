#include <cstdlib>
#include <opencv2/opencv.hpp>

#include "utils/glog.hpp"


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");
  glog::Logger logger(argv);

  cv::Mat_<uchar> m(1000, 1000);

  glog::Timer timer;
  for (int i = 0; i < m.rows; i++) {
    for (int j = 0; j < m.cols; j++) {
      m(i, j) = i + j;
    }
  }
  LOG(INFO) << "Time: " << timer.elapsed() << " s";

  return 0;
}
