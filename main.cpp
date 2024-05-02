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
  camera::KannalaBrandt8 cam({190.97847715128717, 190.9733070521226, 254.93170605935475, 256.8974428996504,
                              0.0034823894022493434, 0.0007150348452162257, -0.0020532361418706202, 0.00020293673591811182});
  for (float x = 0; x < 640; x++) {
    for (float y = 0; y < 480; y++) cam.unproject({x, y});
  }

  return 0;
}
