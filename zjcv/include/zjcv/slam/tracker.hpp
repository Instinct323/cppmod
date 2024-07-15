#ifndef ZJCV__SLAM__TRACKER_HPP
#define ZJCV__SLAM__TRACKER_HPP

#include <yaml-cpp/yaml.h>

#include "zjcv/camera.hpp"
#include "zjcv/imu.hpp"

namespace slam {

class Tracker;

}

#define ZJCV_SLAM_TRACKER_MEMBER \
    typedef std::shared_ptr<Tracker> Ptr; \
    System *mpSystem; \
    const IMU::Device::Ptr mpIMU; \
    const camera::Base::Ptr mpCam0, mpCam1; \
    IMU::Preintegration::Ptr mpIMUpreint; \
    std::shared_ptr<Frame> mpLastFrame, mpCurFrame; \
    Sophus::SE3f T_cam0_cam1;


#define ZJCV_SLAM_TRACKER_CONSTRUCTOR \
    explicit Tracker(System *pSystem, const YAML::Node &cfg) : \
        mpSystem(pSystem), mpIMU(IMU::Device::from_yaml(cfg["imu"])), \
        mpCam0(camera::from_yaml(cfg["cam0"])), mpCam1(camera::from_yaml(cfg["cam1"])) { \
      ASSERT(!is_monocular(), "Not implemented") \
      ASSERT(mpCam0, "Camera0 not found") \
      if (is_stereo()) { \
        ASSERT(mpCam0->get_type() == mpCam1->get_type(), "Camera0 and Camera1 must be the same type") \
        T_cam0_cam1 = mpCam0->T_cam_imu * mpCam1->T_cam_imu.inverse(); \
      } \
    }


#define ZJCV_SLAM_TRACKER_FLAGS \
    inline bool is_inertial() const { return mpIMU != nullptr; } \
    inline bool is_monocular() const { return mpCam1 == nullptr; } \
    inline bool is_stereo() const { return mpCam1 != nullptr; }


#define ZJCV_SLAM_TRACKER_GRAB_IMU \
    void grab_imu(const double &tCurframe, const std::vector<double> &vTimestamp, const std::vector<IMU::Sample> &vSample) { \
      if (!mpIMUpreint) mpIMUpreint = std::make_shared<IMU::Preintegration>(mpIMU.get(), vTimestamp.empty() ? tCurframe : vTimestamp[0]); \
      mpIMUpreint->integrate(tCurframe, vTimestamp, vSample); \
    }


#define ZJCV_SLAM_TRACKER_GRAB_IMAGE \
    void grab_image(const double &timestamp, const cv::Mat &img0, const cv::Mat &img1 = cv::Mat()) { \
      ASSERT(!is_inertial() || timestamp <= mpIMUpreint->mtEnd, "Grab image before IMU data is integrated") \
      ASSERT(is_monocular() == img1.empty(), "Invalid image pair") \
      auto mpTmp = std::make_shared<Frame>(mpSystem, timestamp, img0, img1); \
      mpTmp->process(); \
      mpLastFrame = mpCurFrame; \
      mpCurFrame = mpTmp; \
    }

#endif
