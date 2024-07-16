#ifndef ORBSLAM__FRAME_HPP
#define ORBSLAM__FRAME_HPP

#include "zjcv/slam/system.hpp"

namespace slam {

class System;


class Frame {

public:
    ZJCV_SLAM_FRAME_MEMBER

    ZJCV_SLAM_FRAME_FUNCDECL

    // Features
    cv::Mat mDesc0, mDesc1;
    std::vector<cv::DMatch> mStereoMatches;
};

}

#endif
