#include "utils.hpp"

#include "zjslam/include/camera/kannala_brandt.hpp"
#include "zjslam/include/data.hpp"


int main(int argc, char **argv) {
  Logger logger(argv);
  LOG(INFO) << "CXX standard: " << __cplusplus;

  std::cout.precision(6);

  KannalaBrandt8 cam1({190.97847715128717, 190.9733070521226, 254.93170605935475, 256.8974428996504,
                       0.0034823894022493434, 0.0007150348452162257, -0.0020532361418706202, 0.00020293673591811182});

  TumRgbd loader("/home/workbench/rgbd_dataset_freiburg1_desk2");
  TumRgbd::Timestamps timestamps;
  TumRgbd::ImgFiles img_files;
  loader.loadImage(timestamps, img_files);

  LOG(INFO) << loader;

  return 0;
}
