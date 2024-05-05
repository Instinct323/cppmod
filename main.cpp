#include <cstdlib>

#include "zjslam/include/logging.hpp"
#include "zjslam/include/dataset/tum_vi.hpp"
#include "zjslam/include/camera.hpp"
#include "zjslam/include/utils.hpp"
#include "zjslam/include/imu_type.hpp"


void fisheye_test() {
  // 加载 TUM-VI 数据集 相机参数
  dataset::TumVI tumvi("/home/workbench/data/dataset-corridor4_512_16/dso");
  YAML::Node cfg = tumvi.loadCfg();
  auto cam(camera::fromYAML<camera::KannalaBrandt8>(cfg["cam0"]));

  // 加载图像列表, 读取第一张图像
  GrayLoader loader;
  dataset::Timestamps vTimestamps;
  dataset::Filenames vFilename;
  tumvi.loadImage(vTimestamps, vFilename);
  cv::Mat img = loader(vFilename[0]), dst1;

  // 显示原始图像, 以及去畸变后的图像
  cv::imshow("Origin", img);
  cam->drawNormalizedPlane(img, img);
  cv::imshow("NormalizedPlane", img);
  cv::waitKey(0);
}


void pinhole_test() {
  dataset::TumVI tumvi("/home/workbench/data/dataset-corridor4_512_16/dso");
  YAML::Node cfg = tumvi.loadCfg();

  std::vector<float> intrinsics = {458.654, 457.296, 367.215, 248.375};
  std::vector<float> distortion = {-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};

  cv::Mat img = cv::imread("/home/workbench/data/distorted.png"), dst1;
  camera::Pinhole cam(img.size(), intrinsics, distortion, YAML::toSE3d(cfg["cam0"]["T_cam_imu"]));

  // 显示原始图像, 以及去畸变后的图像
  cv::imshow("Origin", img);
  cam.drawNormalizedPlane(img, img);
  cv::imshow("NormalizedPlane", img);
  cv::waitKey();
}


void draft(){
  dataset::TumVI tumvi("/home/workbench/data/dataset-corridor4_512_16/dso");
  IMU::Device::Ptr dev = IMU::Device::fromYAML(tumvi.loadCfg("imu_config.yaml"));
  std::cout << dev->covWalk.diagonal() << std::endl;
}


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");

  Logger logger(argv);
  LOG(INFO) << "CXX standard: " << __cplusplus;

  fisheye_test();

  return 0;
}
