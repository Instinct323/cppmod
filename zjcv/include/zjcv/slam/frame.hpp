#ifndef ZJCV__SLAM__FRAME_HPP
#define ZJCV__SLAM__FRAME_HPP

#include <opencv2/opencv.hpp>

#include "utils/cv.hpp"
#include "utils/sophus.hpp"
#include "zjcv/imu.hpp"
#include "zjcv/zjcv.hpp"

#define STEREO_OBS_COS_THRESH 0.9998

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
    ZJCV_BUILTIN cv::Mat mImg0, mImg1;

    // Features
    ZJCV_BUILTIN std::vector<Eigen::Vector3f> mvUnprojs0, mvUnprojs1;
    ZJCV_BUILTIN std::vector<std::shared_ptr<Mappoint>> mvpMappts;

    // Processed data
    ZJCV_BUILTIN Ptr mpRefFrame;
    ZJCV_BUILTIN int mnMappts = 0;
    ZJCV_BUILTIN IMU::MovingPose mPose;

    ZJCV_BUILTIN explicit Frame(System *pSystem, const double &timestamp, const cv::Mat &img0, const cv::Mat &img1);

    ZJCV_BUILTIN bool is_keyframe() { return mIdKey > 0; }

    // 根据匹配, 三角化地图点
    ZJCV_BUILTIN int stereo_triangulation(const Frame::Ptr &shared_this, const std::vector<cv::DMatch> &stereo_matches = {});

    // 根据匹配, 合并两帧的地图点
    ZJCV_BUILTIN int connect_frame(Ptr &shared_this, Ptr &ref, std::vector<cv::DMatch> &ref2this);

    // 标记为关键帧
    ZJCV_BUILTIN void mark_keyframe();

    // 世界坐标
    ZJCV_BUILTIN const Eigen::Vector3f &get_pos() { return mPose.T_imu_world.translation(); }

    // pangolin 绘图
    ZJCV_BUILTIN void show_in_opengl(float imu_size, const float *imu_color, bool show_cam = false);

    ZJCV_CUSTOM void process();

    ZJCV_CUSTOM void draw();

#ifdef ZJCV_ORB_SLAM
    std::vector<cv::KeyPoint> mvKps0, mvKps1;
    cv::Mat mDesc0, mDesc1;
    std::vector<cv::DMatch> mRefToThisMatches;

    bool monocular_init(float &ref_radio, Ptr pCurFrame);

    void match_stereo(int lap_cnt0);

    bool match_previous(float &ref_radio);
#endif

};

}

}

#endif
