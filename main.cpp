#include <cstdlib>

#include "zjslam/include/logging.hpp"
#include "zjslam/include/dataset/tum_vi.hpp"
#include "zjslam/include/camera.hpp"
#include "zjslam/include/utils.hpp"


void dataset_test() {
  dataset::TumVI tumvi("/home/workbench/data/dataset-corridor4_512_16/dso");
  YAML::Node cfg = tumvi.loadCfg()["cam0"];

  dataset::Timestamps vTimestamps;
  dataset::Filenames vFilename;
  tumvi.loadImage(vTimestamps, vFilename);
}


void pinhole_test() {
  // cv::stereoRectify();
  ImageLoader loader;

  dataset::TumVI tumvi("/home/workbench/data/dataset-corridor4_512_16/dso");
  YAML::Node cfg = tumvi.loadCfg()["cam0"];
  camera::KannalaBrandt8::Ptr cam(camera::fromYAML<camera::KannalaBrandt8>(cfg));
  std::cout << cam->T_cam_imu.matrix3x4() << std::endl;

  dataset::Timestamps vTimestamps;
  dataset::Filenames vFilename;
  tumvi.loadImage(vTimestamps, vFilename);
  cv::Mat img = loader(vFilename[0]), dst1;

  /*cv::Mat img = cv::imread("/home/workbench/data/distorted.png"), dst1;
  camera::Pinhole cam(
      img.size(),
      {458.654, 457.296, 367.215, 248.375},
      {-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05}
  );*/
  cv::imshow("0", img);

  cam->undistortShow(img);

  cv::waitKey(0);
}


void draft() {
  dataset::TumVI tumvi("/home/workbench/data/dataset-calib-cam2_512_16/dso");
  dataset::Timestamps vTimestamps;
  dataset::Filenames vFilename;
  tumvi.loadImage(vTimestamps, vFilename);
}


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");

  Logger logger(argv);
  LOG(INFO) << "CXX standard: " << __cplusplus;

  pinhole_test();

  return 0;
}
