#ifndef ZJCV__SLAM__TRACKER_HPP
#define ZJCV__SLAM__TRACKER_HPP

#include <sophus/se3.hpp>
#include <yaml-cpp/yaml.h>

#include "zjcv/camera.hpp"
#include "zjcv/imu.hpp"

#ifdef ZJCV_ORB_SLAM

#include "utils/orb.hpp"

#endif

namespace slam {


enum TrackState {
    OK = 0,
    RECENTLY_LOST = 1,
    LOST = 2,
};


class Tracker {

public:
    ZJCV_BUILTIN typedef std::shared_ptr<Tracker> Ptr;

    ZJCV_BUILTIN System *mpSystem;

    // Config
    ZJCV_BUILTIN const int MIN_MATCHES, MAX_MATCHES;
    ZJCV_BUILTIN const float KEY_MATCHES_RADIO;
    ZJCV_BUILTIN const double LOST_TIMEOUT;

    // Device
    ZJCV_BUILTIN const camera::Base::Ptr mpCam0, mpCam1;
    ZJCV_BUILTIN const float mDepthInvScale = -1;
    ZJCV_BUILTIN const IMU::Device::Ptr mpIMU;

    // Frames
    ZJCV_BUILTIN std::shared_ptr<feature::Frame> mpLastFrame, mpCurFrame, mpRefFrame;
    ZJCV_BUILTIN IMU::Preintegration::Ptr mpIMUpreint;
    ZJCV_BUILTIN TrackState mState = LOST;

    ZJCV_BUILTIN explicit Tracker(System *pSystem, const YAML::Node &cfg);

    ZJCV_BUILTIN inline bool is_monocular() const { return mpCam1 == nullptr; }

    ZJCV_BUILTIN inline bool is_stereo() const { return mpCam1 != nullptr; }

    ZJCV_BUILTIN inline bool is_depth() const { return mDepthInvScale > 0; }

    ZJCV_BUILTIN inline bool is_inertial() const { return mpIMU != nullptr; }

    ZJCV_BUILTIN inline void to_depth(cv::Mat &src) { src.convertTo(src, CV_32F, mDepthInvScale); }

    ZJCV_BUILTIN void grab_imu(const double &tCurframe, const std::vector<double> &vTimestamp, const std::vector<IMU::Sample> &vSample);

    ZJCV_BUILTIN void grab_image(const double &timestamp, const cv::Mat &img0, const cv::Mat &img1 = cv::Mat());

    ZJCV_BUILTIN void switch_state(TrackState state);

    ZJCV_CUSTOM void run();

#ifdef ZJCV_ORB_SLAM
    int GRID_SIZE;
    ORB::Extractor::Ptr mpExtractor0, mpExtractor1;
    ORB::Matcher::Ptr mpMatcher;

    void reload(const YAML::Node &cfg) {
      GRID_SIZE = cfg["grid_size"].as<int>();
      mpExtractor0 = ORB::Extractor::from_yaml(cfg["orb0"]);
      mpExtractor1 = ORB::Extractor::from_yaml(cfg["orb1"]);
      mpMatcher = ORB::Matcher::from_yaml(cfg["matcher"]);
      assert(mpExtractor0 && "Extractor0 not found");
      assert(mpMatcher && "Matcher not found");
      assert((!mpCam1) == (!mpExtractor1) && "miss match between Camera1 and Extractor1");
    }

#endif

};

}

#endif
