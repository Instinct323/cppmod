#ifndef ZJCV__SLAM__FRAME_HPP
#define ZJCV__SLAM__FRAME_HPP

#include <map>
#include <opencv2/opencv.hpp>

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
    ZJCV_BUILTIN static bool mbNegDepth;
    ZJCV_BUILTIN System *mpSystem;

    // Origin data
    ZJCV_BUILTIN size_t mId, mIdKey = 0, mIdVex = SIZE_MAX;
    ZJCV_BUILTIN const double mTimestamp;
    ZJCV_BUILTIN cv::Mat mImg0, mImg1;

    // Features
    ZJCV_BUILTIN std::vector<Eigen::Vector3f> mvUnprojs0, mvUnprojs1;
    ZJCV_BUILTIN std::map<int, std::shared_ptr<Mappoint>> mmpMappts;

    // Processed data
    ZJCV_BUILTIN Ptr mpRefFrame;
    ZJCV_BUILTIN int mnMappts = 0;
    ZJCV_BUILTIN IMU::MovingPose mPose;
    ZJCV_BUILTIN Sophus::Joint mJoint;

    ZJCV_BUILTIN explicit Frame(System *pSystem, const double &timestamp, const cv::Mat &img0, const cv::Mat &img1);

    ZJCV_BUILTIN bool is_keyframe() { return mIdKey > 0; }

    template<typename T>
    ZJCV_BUILTIN bool get_depth(int i, T &depth) const {
      if (mbNegDepth) {
        if (mvUnprojs0[i](2) > 0) return false;
        depth = - mvUnprojs0[i][2];
      } else {
        if (mvUnprojs0[i](2) < 0) return false;
        depth = mvUnprojs0[i][2];
      }
      return true;
    }

    // Triangulate the Mappoints according to the match
    ZJCV_BUILTIN int stereo_triangulation(const Ptr &shared_this, const std::vector<cv::DMatch> &left2right = {});

    // Generate the Mappoints
    ZJCV_BUILTIN int rgbd_init(const Ptr &shared_this);

    // Merge the Mappoints of the two Frames according to the match
    ZJCV_BUILTIN int connect_frame(Ptr &shared_this, Ptr &ref, std::vector<cv::DMatch> &ref2this);

    // Marked as a key Frame
    ZJCV_BUILTIN void mark_keyframe();

    // World position
    ZJCV_BUILTIN const Eigen::Vector3f &get_pos() { return mPose.T_imu_world.translation(); }

    // Show in OpenGL
    ZJCV_BUILTIN void show_in_opengl(float imu_size, const float *imu_color, bool show_cam = false);

    // Update the pose
    ZJCV_BUILTIN void update_pose() { mPose.set_pose(mJoint.get()); }

    ZJCV_CUSTOM void process();

#ifdef ZJCV_ORB_SLAM
    std::vector<cv::KeyPoint> mvKps0, mvKps1;
    cv::Mat mDesc0, mDesc1;
    std::vector<cv::DMatch> mRefToThisMatches;

    bool monocular_init(float &ref_radio, Ptr pCurFrame);

    void match_stereo(int lap_cnt0);

    bool match_previous(float &ref_radio);

    void draw();
#endif
};

}

}

#endif
