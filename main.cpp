#include "utils/file.hpp"
#include "utils/glog.hpp"
#include "utils/sophus.hpp"


int main(int argc, char** argv) {
  glog::Logger logger(argv);

  static std::string file = "/media/tongzj/Data/Workbench/Lab/slambook2/ch5/rgbd/pose.txt";
  std::vector<Sophus::SE3f> vPoses;

  TXT::row_mapping(
    file, [&vPoses](const std::string& line) {
      std::istringstream iss(line);
      float tx, ty, tz, qx, qy, qz, qw;
      // timestamp tx ty tz qx qy qz qw
      iss >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
      vPoses.emplace_back(Eigen::Quaternionf(qw, qx, qy, qz), Eigen::Vector3f(tx, ty, tz));
      LOG(INFO) << vPoses.back();
    }
  );

  return 0;
}
