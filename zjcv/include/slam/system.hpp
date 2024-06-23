#ifndef ZJCV__SLAM__SYSTEM_HPP
#define ZJCV__SLAM__SYSTEM_HPP

#include "tracker.hpp"

namespace slam {


class System {

public:
    Tracker::Ptr mpTracker;

    explicit System(YAML::Node cfg) : mpTracker(new Tracker(cfg, this)) {};

    // Tracking
    void GrabMono(const double &timestamp, const cv::Mat &img0,
                  const std::vector<IMU::Sample> &vImu = std::vector<IMU::Sample>()) {
      mpTracker->GrabImageAndImu(timestamp, img0, cv::Mat(), vImu);
    }

    void GrabStereo(const double &timestamp, const cv::Mat &img0, const cv::Mat &img1,
                    const std::vector<IMU::Sample> &vImu = std::vector<IMU::Sample>()) {
      mpTracker->GrabImageAndImu(timestamp, img0, img1, vImu);
    }
};

}

#endif
