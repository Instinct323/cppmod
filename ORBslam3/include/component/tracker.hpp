#ifndef ORBSLAM__SLAM__TRACKER_HPP
#define ORBSLAM__SLAM__TRACKER_HPP

#include <yaml-cpp/yaml.h>

#include "frame.hpp"
#include "utils/orb.hpp"
#include "zjcv/camera.hpp"
#include "zjcv/imu.hpp"

namespace slam {

class Frame;

class System;


class Tracker {

public:
    typedef std::shared_ptr<Tracker> Ptr;

    System *mpSystem;

    // Devices
    camera::Base::Ptr mpCam0, mpCam1;
    ORB::Extractor::Ptr mpExtractor0, mpExtractor1;

    // Frames
    Frame::Ptr mpCurFrame, mpLastFrame;

    explicit Tracker(System *pSystem, YAML::Node cfg);

    Tracker(const Tracker &) = delete;

    void grab_image(const double &timestamp, const cv::Mat &img0, const cv::Mat &img1 = cv::Mat());
};

}

#endif
