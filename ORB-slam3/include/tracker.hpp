#ifndef ORBSLAM__TRACKER_HPP
#define ORBSLAM__TRACKER_HPP

#include <yaml-cpp/yaml.h>

#include "frame.hpp"
#include "utils/orb.hpp"
#include "zjcv/camera.hpp"
#include "zjcv/imu.hpp"

namespace slam {

class System;


class Tracker {

public:
    typedef std::shared_ptr<Tracker> Ptr;

    const System *mpSystem;

    // Devices
    const camera::Base::Ptr mpCam0, mpCam1;
    const ORB::Extractor::Ptr mpExtractor0, mpExtractor1;

    // IMU
    const IMU::Device::Ptr mpIMU;
    IMU::Preintegration::Ptr mpIMUpreint;

    // Frames
    Frame::Ptr mpCurFrame, mpLastFrame;

    explicit Tracker(System *pSystem, YAML::Node cfg);

    Tracker(const Tracker &) = delete;

    inline bool is_monocular() const { return mpCam1 == nullptr; }

    inline bool is_stereo() const { return mpCam1 != nullptr; }

    inline bool is_inertial() const { return mpIMU != nullptr; }

    // Grab
    void grab_image(const double &timestamp, const cv::Mat &img0, const cv::Mat &img1 = cv::Mat());

    void grab_imu(const double &tCurframe, const std::vector<double> &vTimestamp, const std::vector<IMU::Sample> &vSample) {
      if (!mpIMUpreint) mpIMUpreint = std::make_shared<IMU::Preintegration>(mpIMU.get(), vTimestamp.empty() ? tCurframe : vTimestamp[0]);
      mpIMUpreint->integrate(tCurframe, vTimestamp, vSample);
    }
};

}

#endif
