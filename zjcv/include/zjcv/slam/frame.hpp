#ifndef ZJCV__SLAM__FRAME_HPP
#define ZJCV__SLAM__FRAME_HPP

#include <opencv2/opencv.hpp>

#include "zjcv/imu.hpp"

namespace slam {

class Frame;

class Mappoint;

}


#define ZJCV_SLAM_FRAME_MEMBER \
    typedef std::shared_ptr<slam::Frame> Ptr; \
    slam::System *mpSystem; \
    IMU::MovingPose mPose; \
    const double mTimestamp; \
    const cv::Mat mImg0, mImg1; \
    std::vector<cv::KeyPoint> mvKps0, mvKps1; \
    std::vector<std::shared_ptr<slam::Mappoint>> mvpMappoints;


#define ZJCV_SLAM_FRAME_FUNCDECL \
    explicit Frame(slam::System *pSystem, const double &timestamp, const cv::Mat &img0, const cv::Mat &img1); \
    void process();


#define ZJCV_SLAM_FRAME_IMPL \
    slam::Frame::Frame(slam::System *pSystem, const double &timestamp, const cv::Mat &img0, const cv::Mat &img1 \
    ) : mpSystem(pSystem), mTimestamp(timestamp), mImg0(img0), mImg1(img1) {}

#endif
