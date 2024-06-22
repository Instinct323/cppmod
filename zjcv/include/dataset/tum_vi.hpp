#ifndef ZJCV__DATASET__TUM_VI_HPP
#define ZJCV__DATASET__TUM_VI_HPP

#include "base.hpp"

namespace dataset {


/**
 * @brief TUM-VI https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset
 */
class TumVI : public Base {

public:
    // 继承构造函数
    // e.g., ~/dataset-corridor4_512_16/dso
    using Base::Base;

    // cam0, cam1
    void loadImage(Timestamps &vTimestamps, Filenames &vFilename, const std::string &folder = "cam0") {
      assert(vTimestamps.empty() && vFilename.empty());
      std::string root = mPath + folder + "/";
      TXT::rowMapping(
          root + "times.txt",
          [&root, &vTimestamps, &vFilename](const std::string &line) {
              std::istringstream iss(line);
              std::string filename;
              double timestamp;
              // filename timestamp
              iss >> filename >> timestamp;
              vTimestamps.push_back(timestamp);
              vFilename.push_back(root + "images/" + filename + ".png");
          });
    }

    // imu.txt
    void loadIMU(Timestamps &vTimestamps, IMUsamples &vImu, const std::string &file = "imu.txt") {
      assert(vTimestamps.empty() && vImu.empty());
      TXT::rowMapping(
          mPath + file,
          [&vTimestamps, &vImu](const std::string &line) {
              std::istringstream iss(line);
              double timestamp;
              float wx, wy, wz, ax, ay, az;
              // timestamp wx wy wz ax ay az
              iss >> timestamp >> wx >> wy >> wz >> ax >> ay >> az;
              vTimestamps.push_back(timestamp / 1e9);
              vImu.emplace_back(ax, ay, az, wx, wy, wz);
          });
    }

    // camchain.yaml, imu_config.yaml
    YAML::Node loadCfg(const std::string &file = "camchain.yaml") { return YAML::LoadFile(mPath + file); }
};

}

#endif
