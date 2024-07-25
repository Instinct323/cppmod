#ifndef ZJCV__SLAM__TRACKER_HPP
#define ZJCV__SLAM__TRACKER_HPP

#include <sophus/se3.hpp>
#include <yaml-cpp/yaml.h>

#include "zjcv/camera.hpp"
#include "zjcv/imu.hpp"

namespace slam {


class Tracker {

public:
    ZJCV_BUILTIN typedef std::shared_ptr<Tracker> Ptr;

    ZJCV_BUILTIN System *mpSystem;

    // 配置参数
    ZJCV_BUILTIN const int MIN_MATCHES, MAX_MATCHES;

    // 设备信息
    ZJCV_BUILTIN const camera::Base::Ptr mpCam0, mpCam1;
    ZJCV_BUILTIN Sophus::SE3f T_cam0_cam1;
    ZJCV_BUILTIN const IMU::Device::Ptr mpIMU;

    // 帧信息
    ZJCV_BUILTIN std::shared_ptr<Frame> mpLastFrame, mpCurFrame, mpRefFrame;
    ZJCV_BUILTIN IMU::Preintegration::Ptr mpIMUpreint;

    ZJCV_BUILTIN explicit Tracker(System *pSystem, const YAML::Node &cfg);

    ZJCV_BUILTIN inline bool is_inertial() const { return mpIMU != nullptr; }

    ZJCV_BUILTIN inline bool is_monocular() const { return mpCam1 == nullptr; }

    ZJCV_BUILTIN inline bool is_stereo() const { return mpCam1 != nullptr; }

    ZJCV_BUILTIN void grab_imu(const double &tCurframe, const std::vector<double> &vTimestamp, const std::vector<IMU::Sample> &vSample);

    ZJCV_BUILTIN void grab_image(const double &timestamp, const cv::Mat &img0, const cv::Mat &img1 = cv::Mat());

    ZJCV_CUSTOM void run();

#ifdef ZJCV_ORB_SLAM
    // Extractors
    ORB::Extractor::Ptr mpExtractor0, mpExtractor1;

    void reload(const YAML::Node &cfg) {
      mpExtractor0 = ORB::Extractor::from_yaml(cfg["orb0"]);
      mpExtractor1 = ORB::Extractor::from_yaml(cfg["orb1"]);
      ASSERT(mpExtractor0, "Extractor0 not found")
      ASSERT((!mpCam1) == (!mpExtractor1), "miss match between Camera1 and Extractor1")
    }
#endif

};

}

#endif
