#ifndef ZJCV__SLAM__FRAME_HPP
#define ZJCV__SLAM__FRAME_HPP

#include <opencv2/opencv.hpp>

#include "zjcv/imu.hpp"

namespace slam {

class Frame;

}

#define ZJCV_SLAM_FRAME_MEMBER \
    typedef std::shared_ptr<Frame> Ptr; \
    System *mpSystem; \
    const double mTimestamp; \
    const cv::Mat mImg0, mImg1; \
    IMU::MovingPose mPose;


#define ZJCV_SLAM_FRAME_CONSTRUCTOR \
    explicit Frame(System *pSystem, const double &timestamp, const cv::Mat &img0, const cv::Mat &img1 \
    ) : mpSystem(pSystem), mTimestamp(timestamp), mImg0(img0), mImg1(img1) {} \

#endif
