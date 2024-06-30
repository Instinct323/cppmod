#ifndef ZJCV__SLAM__SYSTEM_HPP
#define ZJCV__SLAM__SYSTEM_HPP

#include <atomic>

#include "tracker.hpp"
#include "utils/parallel.hpp"
#include "utils/math.hpp"
#include "viewer.hpp"

namespace slam {


class System {

public:
    Tracker::Ptr mpTracker;
    Viewer::Ptr mpViewer;

    // Status
    std::atomic_bool mbRunning = false;

    explicit System(YAML::Node cfg
    ) : mpTracker(new Tracker(this, cfg)), mpViewer(new Viewer(this)) {};

    System(const System &) = delete;

    // Daemons
    void run() {
      mbRunning = true;
      parallel::thread_pool.emplace(0, &Viewer::run, mpViewer);
    }

    void stop() {
      mbRunning = false;
      parallel::thread_pool.join();
    }

    // Tracking
    void grad_mono(const double &timestamp, const cv::Mat &img0,
                   const std::vector<IMU::Sample> &vImu = std::vector<IMU::Sample>()) {
      mpTracker->grad_image_and_imu(timestamp, img0, cv::Mat(), vImu);
    }

    void grad_stereo(const double &timestamp, const cv::Mat &img0, const cv::Mat &img1,
                     const std::vector<IMU::Sample> &vImu = std::vector<IMU::Sample>()) {
      mpTracker->grad_image_and_imu(timestamp, img0, img1, vImu);
    }
};

}

#endif
