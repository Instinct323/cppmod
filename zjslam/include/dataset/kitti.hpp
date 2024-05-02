#ifndef ZJSLAM__DATASET__KITTI_HPP
#define ZJSLAM__DATASET__KITTI_HPP

#include "base.hpp"

namespace dataset {

    /**
     * @brief KITTI https://www.cvlibs.net/datasets/kitti/eval_odometry.php
     */
    class Kitti : public Base {
        std::string mId;

    public:
        typedef Eigen::Matrix<double, 3, 4> Matrix34d;

        // e.g., ~/dataset
        explicit Kitti(const std::string &path, int seqid) : Base(path) {
          std::stringstream ss;
          ss << std::setw(2) << std::setfill('0') << seqid;
          mId = ss.str();
        }

        // times.txt
        void loadTimestamps(Timestamps &vTimestamps, const std::string &file = "times.txt") {
          assert(vTimestamps.empty());
          processTxt(mPath + "sequences/" + mId + "/" + file,
                     [&vTimestamps](const std::string &line) {
                         vTimestamps.push_back(std::stod(line));
                     });
        }

        // fixme: e.g., image_0
        void loadImage(Filenames &vFilename, const std::string &folder = "image_0") {
          assert(vFilename.empty());
          std::string img_path = mPath + "sequences/" + mId + "/" + folder;
          for (const auto &entry: std::filesystem::directory_iterator(img_path)) {
            vFilename.push_back(entry.path());
          }
        }

        void loadPoses(Poses &vPoses) {
          assert(vPoses.empty());
          processTxt(mPath + "poses/" + mId + ".txt",
                     [&vPoses](const std::string &line) {
                         std::istringstream iss(line);
                         Matrix34d pose = Matrix34d::Identity();
                         for (int i = 0; i < 3; i++) {
                           for (int j = 0; j < 4; j++) iss >> pose(i, j);
                         }
                         vPoses.emplace_back(Eigen::Quaterniond(pose.block<3, 3>(0, 0)), pose.block<3, 1>(0, 3));
                     });
        }

        void savePoses(Poses &vPoses) {
          std::ofstream f(mPath + mId + ".txt");
          for (const auto &pose: vPoses) {
            Matrix34d p = pose.matrix3x4();
            for (int i = 0; i < p.size(); i++) {
              f << *(p.data() + i) << " ";
            }
            f << std::endl;
          }
          f.close();
        }
    };
}

#endif
