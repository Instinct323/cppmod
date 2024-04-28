#include "utils.hpp"

#include "zjslam/include/camera/pinhole.hpp"
#include "zjslam/include/camera/kannala_brandt.hpp"


int main(int argc, char **argv) {
  Logger logger(argv);
  LOG(INFO) << "CXX standard: " << __cplusplus;

  std::cout.precision(2);

  KannalaBrandt8 cam1({190.97847715128717, 190.9733070521226, 254.93170605935475, 256.8974428996504,
                       0.0034823894022493434, 0.0007150348452162257, -0.0020532361418706202, 0.00020293673591811182});

  KannalaBrandt8 cam2({10.97847715128717, 10.9733070521226, 254.93170605935475, 256.8974428996504,
                       0.0034823894022493434, 0.0007150348452162257, -0.0020532361418706202, 0.00020293673591811182});
  for (int x = 0; x < 640; x++) {
    for (int y = 0; y < 480; y++) {
      LOG(INFO) << cam2.unprojectEig(cv::Point2f(x, y));
    }
  }

  return 0;
}
