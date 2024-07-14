#ifndef ZJCV__SLAM__FRAME_HPP
#define ZJCV__SLAM__FRAME_HPP

#include <opencv2/opencv.hpp>

namespace slam {


template<typename System>
class FrameBase {

public:
    System *mpSystem;

    // Origin Data
    const double mTimestamp;
    const cv::Mat mImg0, mImg1;

    // Status
    Sophus::SE3f T_imu_world;

    explicit FrameBase(System *pSystem, const double &timestamp, const cv::Mat &img0, const cv::Mat &img1
    ) : mpSystem(pSystem), mTimestamp(timestamp), mImg0(img0), mImg1(img1) {};

    FrameBase(const FrameBase &) = delete;

    virtual void process() = 0;
};

}

#endif
