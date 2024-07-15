#ifndef ORBSLAM__FRAME_HPP
#define ORBSLAM__FRAME_HPP

#include "utils/eigen.hpp"
#include "utils/orb.hpp"
#include "zjcv/camera.hpp"
#include "zjcv/slam/system.hpp"

namespace slam {

class System;


class Frame {

public:
    ZJCV_SLAM_FRAME_MEMBER

    ZJCV_SLAM_FRAME_CONSTRUCTOR

    // Features
    ORB::KeyPoints mvKps0, mvKps1;
    cv::Mat mDesc0, mDesc1;
    std::vector<cv::DMatch> mStereoMatches;

    void process();
};

}

#endif
