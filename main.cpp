#include <cstdlib>

#include "utils/glog.hpp"
#include "utils/pangolin.hpp"
#include "zjcv/dataset/tum_vi.hpp"


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");
  glog::Logger logger(argv);

  YAML::Node cfg = YAML::LoadFile("/home/workbench/cppmod/ORB-slam3/example/TUM-VI.yaml");
  dataset::TumVI tum_vi(cfg["dataset"].as<std::string>());

  dataset::Timestamps vTsImg, vTsPose;
  dataset::Filenames vFilenames;
  dataset::Poses vPoses;
  tum_vi.load_image(vTsImg, vFilenames);
  tum_vi.load_pose(vTsPose, vPoses);
  LOG(INFO) << "Image: " << vTsImg.size() << ", Pose: " << vTsPose.size();

  pangolin::plot_trajectory(cfg["viewer"], vTsImg, vFilenames, vTsPose, vPoses);

  return 0;
}
