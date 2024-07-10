#ifndef ORBSLAM__FRAME_HPP
#define ORBSLAM__FRAME_HPP

#include "utils/orb.hpp"
#include "zjcv/imu.hpp"

namespace slam {


template<typename Tracker>
class FrameBase {

public:
    typedef std::shared_ptr<FrameBase> Ptr;

    const Tracker *mpTracker;

    // Origin Data
    const double mTimestamp;
    const cv::Mat mImg0, mImg1;

    FrameBase(Tracker *pTracker, const double &timestamp, const cv::Mat &img0, const cv::Mat &img1
    ) : mpTracker(pTracker), mTimestamp(timestamp), mImg0(img0), mImg1(img1) {};

    FrameBase(const FrameBase &) = delete;
};

}

#endif
