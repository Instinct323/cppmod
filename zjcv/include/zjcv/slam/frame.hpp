#ifndef ZJCV__SLAM__FRAME_HPP
#define ZJCV__SLAM__FRAME_HPP

#include <opencv2/opencv.hpp>

#include "utils/cv.hpp"
#include "zjcv/imu.hpp"

namespace slam {

namespace feature {

class Mappoint;

// 基于特征点
class Frame {

public:
    ZJCV_BUILTIN typedef std::shared_ptr<Frame> Ptr;

    ZJCV_BUILTIN System *mpSystem;
    ZJCV_BUILTIN const double mTimestamp;
    ZJCV_BUILTIN const cv::Mat mImg0, mImg1;

    ZJCV_BUILTIN IMU::MovingPose mPose;
    ZJCV_BUILTIN std::vector<cv::KeyPoint> mvKps0, mvKps1;
    ZJCV_BUILTIN std::vector<std::shared_ptr<Mappoint>> mvpMappts;

    ZJCV_BUILTIN explicit Frame(System *pSystem, const double &timestamp, const cv::Mat &img0, const cv::Mat &img1);

    ZJCV_BUILTIN ~Frame();

    ZJCV_BUILTIN int init_mappoints(const std::vector<cv::DMatch> &matches = {});

    ZJCV_BUILTIN int connect_frame(Ptr &other, const std::vector<cv::DMatch> &matches);

    ZJCV_CUSTOM void process();

    ZJCV_CUSTOM void draw();

#ifdef ZJCV_ORB_SLAM
    cv::Mat mDesc0, mDesc1;
    std::vector<cv::DMatch> mStereoMatches;
    std::unique_ptr<cv::GridMask> mpGridMask;
#endif

};

}

}

#endif
