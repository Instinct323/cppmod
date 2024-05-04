#include <cstdlib>

#include "zjslam/include/logging.hpp"
#include "zjslam/include/dataset/tum_vi.hpp"
#include "zjslam/include/camera.hpp"
#include "zjslam/include/utils.hpp"


void fisheye_test() {
  // cv::stereoRectify();
  ImageLoader loader;

  dataset::TumVI tumvi("/home/workbench/data/dataset-corridor4_512_16/dso");
  YAML::Node cfg = tumvi.loadCfg();
  camera::Base::Ptr cam(camera::fromYAML<camera::KannalaBrandt8>(cfg["cam0"]));

  dataset::Timestamps vTimestamps;
  dataset::Filenames vFilename;
  tumvi.loadImage(vTimestamps, vFilename);
  cv::Mat img = loader(vFilename[0]), dst1;

  cv::imshow("0", img);
  cam->drawNormalizedPlane(img, img);
  cv::imshow("1", img);

  cv::waitKey(0);
}


void pinhole_test() {
  dataset::TumVI tumvi("/home/workbench/data/dataset-corridor4_512_16/dso");
  YAML::Node cfg = tumvi.loadCfg();

  std::vector<float> intrinsics = {458.654, 457.296, 367.215, 248.375};
  std::vector<float> distortion = {-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};

  cv::Mat img = cv::imread("/home/workbench/data/distorted.png"), dst1;
  camera::Pinhole cam1(img.size(), intrinsics, distortion, YAML::toSE3d(cfg["cam0"]["T_cam_imu"])),
      cam2(img.size(), intrinsics, distortion, YAML::toSE3d(cfg["cam1"]["T_cam_imu"]));

  cam1.stereoRectify(&cam2);

  cv::waitKey();
}


void draft(){

}


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");

  Logger logger(argv);
  LOG(INFO) << "CXX standard: " << __cplusplus;

  draft();

  return 0;
}
