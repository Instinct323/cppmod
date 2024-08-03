#ifndef ZJCV__SLAM__FRAME_HPP
#define ZJCV__SLAM__FRAME_HPP

#include <opencv2/opencv.hpp>

#include "utils/cv.hpp"
#include "utils/sophus.hpp"
#include "zjcv/imu.hpp"
#include "zjcv/zjcv.hpp"

namespace slam {

class System;

// 基于特征点
namespace feature {

class Mappoint;

// 基于特征点
class Frame {

public:
    ZJCV_BUILTIN typedef std::shared_ptr<Frame> Ptr;

    ZJCV_BUILTIN static size_t FRAME_COUNT, KEY_COUNT;
    ZJCV_BUILTIN System *mpSystem;

    // Origin data
    ZJCV_BUILTIN size_t mId, mIdKey = 0, mIdVex = SIZE_MAX;
    ZJCV_BUILTIN const double mTimestamp;
    ZJCV_BUILTIN const cv::Mat mImg0, mImg1;

    // Features
    ZJCV_BUILTIN std::vector<Eigen::Vector3f> mvUnprojs0, mvUnprojs1;
    ZJCV_BUILTIN std::vector<std::shared_ptr<Mappoint>> mvpMappts;

    // Processed data
    ZJCV_BUILTIN IMU::MovingPose mPose;
    ZJCV_BUILTIN Sophus::Joint mJoint;

    ZJCV_BUILTIN explicit Frame(System *pSystem, const double &timestamp, const cv::Mat &img0, const cv::Mat &img1);

    ZJCV_BUILTIN bool is_keyframe() { return mIdKey > 0; }

    // 根据匹配, 三角化地图点
    ZJCV_BUILTIN int stereo_triangulation(const Frame::Ptr &shared_this, const std::vector<cv::DMatch> &stereo_matches = {});

    // 根据匹配, 合并两帧的地图点
    ZJCV_BUILTIN int connect_frame(Ptr &shared_this, Ptr &other, std::vector<cv::DMatch> &matches);

    ZJCV_BUILTIN void mark_keyframe();

    ZJCV_BUILTIN void prune();

    ZJCV_BUILTIN void show_in_pangolin(float imu_size, float mp_size, const float *imu_color, const float *mp_color);

    ZJCV_CUSTOM void process();

    ZJCV_CUSTOM void draw();

#ifdef ZJCV_ORB_SLAM
    std::vector<cv::KeyPoint> mvKps0, mvKps1;
    cv::Mat mDesc0, mDesc1;
    std::vector<cv::DMatch> mStereoMatches;
#endif

};

}

}

#endif
