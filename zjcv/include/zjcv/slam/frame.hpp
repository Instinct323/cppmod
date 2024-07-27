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

    ZJCV_BUILTIN static size_t FRAME_COUNT, KEY_COUNT;
    ZJCV_BUILTIN System *mpSystem;

    // Origin data
    ZJCV_BUILTIN size_t mId, mIdKey = 0;
    ZJCV_BUILTIN const double mTimestamp;
    ZJCV_BUILTIN const cv::Mat mImg0, mImg1;

    // Processed data
    ZJCV_BUILTIN IMU::MovingPose mPose;
    ZJCV_BUILTIN std::vector<Eigen::Vector3f> mvUnprojs0, mvUnprojs1;
    ZJCV_BUILTIN std::vector<std::shared_ptr<Mappoint>> mvpMappts;

    ZJCV_BUILTIN explicit Frame(System *pSystem, const double &timestamp, const cv::Mat &img0, const cv::Mat &img1);

    // 根据匹配, 初始化对应左图关键点的地图点
    ZJCV_BUILTIN int init_mappoints(Ptr &shared_this, const std::vector<cv::DMatch> &matches = {});

    // 根据匹配, 合并两帧的地图点
    ZJCV_BUILTIN int connect_frame(Ptr &shared_this, Ptr &other, std::vector<cv::DMatch> &matches);

    ZJCV_BUILTIN void prune();

    ZJCV_BUILTIN void mark_keyframe();

    ZJCV_CUSTOM void process();

    ZJCV_CUSTOM void draw();

#ifdef ZJCV_ORB_SLAM
    std::vector<cv::KeyPoint> mvKps0, mvKps1;
    cv::Mat mDesc0, mDesc1;
    std::vector<cv::DMatch> mStereoMatches;
    std::unique_ptr<cv::GridDict> mpGridDict;

    void stereo_features(std::vector<cv::DMatch> &matches);

    void unproject_kps();
#endif

};

}

}

#endif
