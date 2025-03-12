#ifndef ZJCV__DATASET__KITTI_HPP
#define ZJCV__DATASET__KITTI_HPP

#include "base.hpp"

namespace dataset {


/**
 * @brief KITTI https://www.cvlibs.net/datasets/kitti/eval_odometry.php
 * @calib P0 (灰左), P1 (灰右), P2 (彩左), P3 (彩右)
 *        根据参数不同分 3 组: (00~02, 13~21); (03,); (04~12)
 */
class Kitti : public Base {
    std::string mId;

public:
    typedef Eigen::Matrix<float, 3, 4> Matrix34f;

    // e.g., ~/dataset
    explicit Kitti(const std::string &path, int seqid) : Base(path) {
      std::stringstream ss;
      ss << std::setw(2) << std::setfill('0') << seqid;
      mId = ss.str();
    }

    Kitti(const Kitti &) = delete;

    // times.txt
    void load_timestamp(Timestamps &vTimestamps, const std::string &file = "times.txt") {
      assert(vTimestamps.empty());
      TXT::row_mapping(
          mPath + "sequences/" + mId + "/" + file,
          [&vTimestamps](const std::string &line) {
              vTimestamps.push_back(std::stod(line));
          });
    }

    // e.g., image_0
    void load_image(Filenames &vFilename, const std::string &folder = "image_0") {
      assert(vFilename.empty());
      std::string img_path = mPath + "sequences/" + mId + "/" + folder;
      for (const auto &entry: std::filesystem::directory_iterator(img_path)) {
        vFilename.push_back(entry.path());
      }
    }

    void load_pose(Poses &vPoses) {
      assert(vPoses.empty());
      TXT::row_mapping(
          mPath + "poses/" + mId + ".txt",
          [&vPoses](const std::string &line) {
              std::istringstream iss(line);
              Matrix34f pose = Matrix34f::Identity();
              for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 4; j++) iss >> pose(i, j);
              }
              vPoses.emplace_back(Eigen::Quaternionf(pose.block<3, 3>(0, 0)), pose.block<3, 1>(0, 3));
          });
    }

    // e.g., P0, P1, P2, P3
    Matrix34f load_calib(std::string id = "P0") {
      std::string file = mPath + "sequences/" + mId + "/calib.txt";
      std::ifstream ifs(file);
      assert(ifs.is_open());
      std::string line;
      while (std::getline(ifs, line)) {
        std::istringstream iss(line);
        std::string token;
        iss >> token;
        if (token == id + ":") {
          Matrix34f P;
          for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) iss >> P(i, j);
          }
          return P;
        }
      }
      throw std::runtime_error("Invalid calib id: " + id);
    }
};

}

#endif
