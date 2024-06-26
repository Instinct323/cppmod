#ifndef ZJCV__SLAM__TRACKER_HPP
#define ZJCV__SLAM__TRACKER_HPP

#include <yaml-cpp/yaml.h>

#include "camera.hpp"
#include "frame.hpp"
#include "imu_type.hpp"
#include "utils/orb.hpp"

namespace slam {

class Frame;

class System;


class Tracker {

public:
    typedef std::shared_ptr<Tracker> Ptr;

    System *mpSystem;

    // Devices
    camera::Base::Ptr mpCam0, mpCam1;
    IMU::Device::Ptr mpIMU;
    ORB::Extractor::Ptr mpExtractor0, mpExtractor1;

    // Frames
    Frame::Ptr mpCurFrame, mpLastFrame;

    explicit Tracker(System *pSystem, YAML::Node cfg);

    void grad_image_and_imu(const double &timestamp, const cv::Mat &img0, const cv::Mat &img1 = cv::Mat(),
                            const std::vector<IMU::Sample> &vImu = std::vector<IMU::Sample>());
};

}

#endif
