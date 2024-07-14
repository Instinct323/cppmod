#ifndef ZJCV__DATASET__TUM_RGBD_HPP
#define ZJCV__DATASET__TUM_RGBD_HPP

#include "base.hpp"

namespace dataset {


/**
 * @brief TUM-RGBD https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download
 */
class TumRGBD : public Base {

public:
    // 继承构造函数
    // e.g., ~/rgbd_dataset_freiburg1_desk2
    using Base::Base;

    // rgb.txt, depth.txt
    void load_image(Timestamps &vTimestamps, Filenames &vFilename, const std::string &file = "rgb.txt") {
      assert(vTimestamps.empty() && vFilename.empty());
      TXT::row_mapping(
          mPath + file,
          [this, &vTimestamps, &vFilename](const std::string &line) {
              std::istringstream iss(line);
              double timestamp;
              std::string filename;
              // timestamp filename
              iss >> timestamp >> filename;
              vTimestamps.push_back(timestamp);
              vFilename.push_back(this->mPath + filename);
          });
    }

    // groundtruth.txt
    void load_poses(Timestamps &vTimestamps, Poses &vPoses, const std::string &file = "groundtruth.txt") {
      assert(vTimestamps.empty() && vPoses.empty());
      TXT::row_mapping(
          mPath + file,
          [&vTimestamps, &vPoses](const std::string &line) {
              std::istringstream iss(line);
              double timestamp;
              float tx, ty, tz, qx, qy, qz, qw;
              // timestamp tx ty tz qx qy qz qw
              iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
              vTimestamps.push_back(timestamp);
              vPoses.emplace_back(Eigen::Quaternionf(qw, qx, qy, qz), Eigen::Vector3f(tx, ty, tz));
          });
    }

    // accelerometer.txt
    void load_accel(Timestamps &vTimestamps, Accels &vAccel, const std::string &file = "accelerometer.txt") {
      assert(vTimestamps.empty() && vAccel.empty());
      TXT::row_mapping(
          mPath + file,
          [&vTimestamps, &vAccel](const std::string &line) {
              std::istringstream iss(line);
              double timestamp;
              float ax, ay, az;
              // timestamp ax ay az
              iss >> timestamp >> ax >> ay >> az;
              vTimestamps.push_back(timestamp);
              vAccel.emplace_back(ax, ay, az);
          });
    }
};

}

#endif
