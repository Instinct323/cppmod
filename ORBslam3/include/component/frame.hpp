#ifndef ZJCV__SLAM__FRAME_HPP
#define ZJCV__SLAM__FRAME_HPP

#include "imu.hpp"
#include "utils/orb.hpp"

namespace slam {

class Tracker;


class Frame {

public:
    typedef std::shared_ptr<Frame> Ptr;

    Tracker *mpTracker;

    // Origin Data
    double mTimestamp;
    cv::Mat mImg0, mImg1;

    // Features
    ORB::KeyPoints mvKps0, mvKps1;
    cv::Mat mDesc0, mDesc1;
    std::vector<cv::DMatch> mStereoMatches;

    Frame(Tracker *pTracker,
          const double &timestamp, const cv::Mat &img0, const cv::Mat &img1);

    Frame(const Frame &) = delete;
};

}

#endif
