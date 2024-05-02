#include <cstdlib>

#include "utils.hpp"
#include "zjslam/include/utils.hpp"
#include "zjslam/include/dataset/tum_vi.hpp"


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");

  Logger logger(argv);
  LOG(INFO) << "CXX standard: " << __cplusplus;

  std::cout.precision(6);

  ImageLoader imgLoader;
  TumVI tumVI("/home/workbench/data/dataset-corridor4_512_16/dso");

  return 0;
}
