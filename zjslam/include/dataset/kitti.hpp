#ifndef ZJSLAM__DATASET__KITTI_HPP
#define ZJSLAM__DATASET__KITTI_HPP

#include "base.hpp"


/**
 * @brief KITTI https://www.cvlibs.net/datasets/kitti/eval_odometry.php
 */
class Kitti : public DatasetBase {
    std::string id;

public:
    // e.g., ~/dataset
    Kitti(const std::string &path, int seqid) : DatasetBase(path) {
      std::stringstream ss;
      ss << std::setw(2) << std::setfill('0') << seqid;
      id = ss.str();
    }

    // times.txt
    void loadTimestamps(Timestamps &vTimestamps, std::string file = "times.txt") {
      processTxt(path + "sequences/" + id + "/" + file,
                 [&vTimestamps](std::string line) {
                     vTimestamps.push_back(std::stod(line));
                 });
    }

    // e.g., image_0
    void loadImage(Filenames &vFilename, std::string folder = "image_0") {
      std::string img_path = path + "sequences/" + id + "/" + folder;
      for (const auto &entry: std::filesystem::directory_iterator(img_path)) {
        vFilename.push_back(entry.path());
      }
    }

    void loadPoses(Poses &vPoses) {
      typedef Eigen::Matrix<double, 3, 4> Matrix34d;
      processTxt(path + "poses/" + id + ".txt",
                 [&vPoses](std::string line) {
                     std::istringstream iss(line);
                     Matrix34d pose = Matrix34d::Identity();
                     for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 4; j++) iss >> pose(i, j);
                     }
                     vPoses.emplace_back(Eigen::Quaterniond(pose.block<3, 3>(0, 0)), pose.block<3, 1>(0, 3));
                 });
    }
};

#endif
