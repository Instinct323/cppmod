#ifndef ORBSLAM__SYSTEM_HPP
#define ORBSLAM__SYSTEM_HPP

#include <atomic>
#include <memory>

#include "tracker.hpp"
#include "utils/parallel.hpp"
#include "utils/math.hpp"
#include "viewer.hpp"

namespace slam {


class System {

public:
    const Tracker::Ptr mpTracker;
    const Viewer::Ptr mpViewer;

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
    void grab_imu(const double &tCurframe, const std::vector<double> &vTimestamp,
                  const std::vector<IMU::Sample> &vSample) { mpTracker->grab_imu(tCurframe, vTimestamp, vSample); }

    void grab_image(const double &timestamp, const cv::Mat &img0,
                    const cv::Mat &img1 = cv::Mat()) { mpTracker->grab_image(timestamp, img0, img1); }
};

}

#endif
