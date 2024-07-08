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
    Tracker::Ptr mpTracker;
    Viewer::Ptr mpViewer;

    // IMU
    IMU::Device::Ptr mpIMU;
    IMU::Preintegration::Ptr mpImuPreint;

    // Status
    std::atomic_bool mbRunning = false;

    explicit System(YAML::Node cfg
    ) : mpTracker(new Tracker(this, cfg)), mpViewer(new Viewer(this)), mpIMU(IMU::Device::from_yaml(cfg["imu"])) {};

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
    void grab_imu(const double &tCurframe, const std::vector<double> &vTimestamp, const std::vector<IMU::Sample> &vSample) {
      if (!mpImuPreint) mpImuPreint = std::make_shared<IMU::Preintegration>(mpIMU.get(), vTimestamp.empty() ? tCurframe : vTimestamp[0]);
      mpImuPreint->integrate(tCurframe, vTimestamp, vSample);
    }

    void grab_mono(const double &timestamp, const cv::Mat &img0) {
      assert(!mpImuPreint || timestamp <= mpImuPreint->mtEnd);
      mpTracker->grab_image(timestamp, img0);
    }

    void grab_stereo(const double &timestamp, const cv::Mat &img0, const cv::Mat &img1) {
      assert(!mpImuPreint || timestamp <= mpImuPreint->mtEnd);
      mpTracker->grab_image(timestamp, img0, img1);
    }
};

}

#endif
