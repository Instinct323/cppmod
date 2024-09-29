#include <cstdlib>

#include "utils/cv.hpp"
#include "utils/glog.hpp"
#include "zjcv/dataset/kitti.hpp"


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");
  glog::Logger logger(argv);

  // config
  YAML::Node cfg = YAML::LoadFile("/home/workbench/cppmod/ORB-slam3/cfg/KITTI.yaml");
  dataset::Filenames image0, image1;

  // 载入并校验数据
  dataset::Kitti kitti(cfg["dataset"].as<std::string>(), cfg["seq_id"].as<int>());
  kitti.load_image(image0, "image_0");
  kitti.load_image(image1, "image_1");

  auto sgbm = cv::StereoSGBM::create(0, 96, 9);
  cv::StereoDepth stereo_depth(sgbm, 718.856, 0.537);
  cv::Mat img_left = cv::imread(image0[0], cv::IMREAD_GRAYSCALE),
      img_right = cv::imread(image1[0], cv::IMREAD_GRAYSCALE),
      disparity, depth;

  stereo_depth.to_depth(img_left, img_right, depth);
  cv::imshow("disparity", depth);
  cv::waitKey(0);

  return 0;
}
