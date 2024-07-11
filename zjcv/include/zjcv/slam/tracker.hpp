#ifndef ZJCV__SLAM__TRACKER_HPP
#define ZJCV__SLAM__TRACKER_HPP

#include <yaml-cpp/yaml.h>

#include "zjcv/camera.hpp"
#include "zjcv/imu.hpp"

namespace slam {


template<typename System>
class TrackerBase {

public:
    typedef typename System::Frame Frame;

    System *mpSystem;

    // Devices
    const camera::Base::Ptr mpCam0, mpCam1;
    const IMU::Device::Ptr mpIMU;
    IMU::Preintegration::Ptr mpIMUpreint;

    // frames
    std::shared_ptr<Frame> mpLastFrame, mpCurFrame;

    // Transform
    Sophus::SE3d T_cam0_cam1;

    explicit TrackerBase(System *pSystem, const YAML::Node &cfg) :
        mpSystem(pSystem),
        mpCam0(camera::from_yaml(cfg["cam0"])), mpCam1(camera::from_yaml(cfg["cam1"])),
        mpIMU(IMU::Device::from_yaml(cfg["imu"])) {
      ASSERT(mpCam0, "Camera0 not found")
      // 双目模式
      if (is_stereo()) {
        ASSERT(mpCam0->get_type() == mpCam1->get_type(), "Camera0 and Camera1 must be the same type")
        T_cam0_cam1 = mpCam0->T_cam_imu * mpCam1->T_cam_imu.inverse();
        // Pinhole: 立体矫正
        /* if (mpCam0->get_type() == camera::CameraType::PINHOLE) {
          static_cast<camera::Pinhole *>(mpCam0.get())->stereo_rectify(static_cast<camera::Pinhole *>(mpCam1.get()));
        } */
      }
    }

    TrackerBase(const TrackerBase &) = delete;

    inline bool is_inertial() const { return mpIMU != nullptr; }

    inline bool is_monocular() const { return mpCam1 == nullptr; }

    inline bool is_stereo() const { return mpCam1 != nullptr; }

    // Grab
    void grab_imu(const double &tCurframe, const std::vector<double> &vTimestamp, const std::vector<IMU::Sample> &vSample) {
      if (!mpIMUpreint) mpIMUpreint = std::make_shared<IMU::Preintegration>(mpIMU.get(), vTimestamp.empty() ? tCurframe : vTimestamp[0]);
      mpIMUpreint->integrate(tCurframe, vTimestamp, vSample);
    }

    void grab_image(const double &timestamp, const cv::Mat &img0, const cv::Mat &img1 = cv::Mat()) {
      ASSERT(!is_inertial() || timestamp <= mpIMUpreint->mtEnd, "Grab image before IMU data is integrated")
      ASSERT(is_monocular() == img1.empty(), "Invalid image pair")
      auto mpTmp = std::make_shared<Frame>(mpSystem, timestamp, img0, img1);
      mpTmp->process();
      mpLastFrame = mpCurFrame;
      mpCurFrame = mpTmp;
    }

    virtual void run() = 0;
};

}

#endif
