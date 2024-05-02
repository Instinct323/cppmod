#ifndef ZJSLAM__DATASET__TUM_VI_HPP
#define ZJSLAM__DATASET__TUM_VI_HPP

#include "base.hpp"


/**
 * @brief TUM-VI https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset
 */
class TumVI : public DatasetBase {

public:
    // e.g., ~/dataset-corridor4_512_16/dso
    TumVI(const std::string &path) : DatasetBase(path) {}

    // cam0, cam1
    void loadImage(Timestamps &vTimestamps, Filenames &vFilename, std::string folder = "cam0") {
      std::string root = mPath + folder + "/";
      processTxt(root + "times.txt",
                 [&root, &vTimestamps, &vFilename](std::string line) {
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
    void loadImu(Timestamps &vTimestamps, ImuSamples &vImu, std::string file = "imu.txt") {
      processTxt(mPath + file,
                 [&vTimestamps, &vImu](std::string line) {
                     std::istringstream iss(line);
                     double timestamp;
                     float wx, wy, wz, ax, ay, az;
                     // timestamp wx wy wz ax ay az
                     iss >> timestamp >> wx >> wy >> wz >> ax >> ay >> az;
                     vTimestamps.push_back(timestamp);
                     vImu.emplace_back(ax, ay, az, wx, wy, wz);
                 });
    }
};

#endif
