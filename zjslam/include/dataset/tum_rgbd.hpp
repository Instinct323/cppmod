#ifndef ZJSLAM__DATASET__TUM_RGBD_HPP
#define ZJSLAM__DATASET__TUM_RGBD_HPP

#include "base.hpp"


/**
 * @brief TUM-RGBD https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download
 */
class TumRGBD : public DatasetBase {

public:
    // e.g., ~/rgbd_dataset_freiburg1_desk2
    TumRGBD(const std::string &path) : DatasetBase(path) {}

    // rgb.txt, depth.txt
    void loadImage(Timestamps &vTimestamps, Filenames &vFilename, std::string file = "rgb.txt") {
      processTxt(mPath + file,
                 [this, &vTimestamps, &vFilename](std::string line) {
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
    void loadPoses(Timestamps &vTimestamps, Poses &vPoses, std::string file = "groundtruth.txt") {
      processTxt(mPath + file,
                 [&vTimestamps, &vPoses](std::string line) {
                     std::istringstream iss(line);
                     double timestamp;
                     double tx, ty, tz, qx, qy, qz, qw;
                     // timestamp tx ty tz qx qy qz qw
                     iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
                     vTimestamps.push_back(timestamp);
                     vPoses.emplace_back(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
                 });
    }

    // accelerometer.txt
    void loadAccel(Timestamps &vTimestamps, Accels &vAccel, std::string file = "accelerometer.txt") {
      processTxt(mPath + file,
                 [&vTimestamps, &vAccel](std::string line) {
                     std::istringstream iss(line);
                     double timestamp;
                     double ax, ay, az;
                     // timestamp ax ay az
                     iss >> timestamp >> ax >> ay >> az;
                     vTimestamps.push_back(timestamp);
                     vAccel.emplace_back(ax, ay, az);
                 });
    }
};


#endif
