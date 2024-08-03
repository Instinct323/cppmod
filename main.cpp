#include <cstdlib>

#include "utils/glog.hpp"
#include "utils/pangolin.hpp"
#include "zjcv/dataset/tum_vi.hpp"

int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");
  glog::Logger logger(argv);

  YAML::Node cfg = YAML::LoadFile("/home/workbench/cppmod/ORB-slam3/example/TUM-VI.yaml");
  dataset::TumVI tum_vi(cfg["dataset"].as<std::string>());
  dataset::Timestamps vImgTs, vPoseTs;
  dataset::Filenames vImgLeft;
  dataset::Poses vPose;

  tum_vi.load_image(vImgTs, vImgLeft, "cam0");
  tum_vi.load_pose(vPoseTs, vPose);
  LOG(INFO) << vPose[0].matrix3x4();

  auto fig = pangolin::Figure::from_yaml(cfg["viewer"]);
  pangolin::Trajectory trace(cfg["viewer"], vPoseTs, vPose);
  for (int i = 0; i < vImgTs.size(); ++i) {
    fig->clear();
    fig->follow(trace.plot(vImgTs[i]));
    fig->draw();
    cv::imshow("TUM-VI", cv::imread(vImgLeft[i]));
    cv::waitKey(1);
  }

  return 0;
}
