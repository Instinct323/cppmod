#ifndef ORBSLAM__TRACKER_HPP
#define ORBSLAM__TRACKER_HPP

#include <yaml-cpp/yaml.h>

#include "utils/orb.hpp"
#include "zjcv/camera.hpp"
#include "zjcv/slam/system.hpp"

namespace slam {

class System;


class Tracker {

public:
    ZJCV_SLAM_TRACKER_MEMBER

    ZJCV_SLAM_TRACKER_FLAGS

    ZJCV_SLAM_TRACKER_GRAB_IMU

    ZJCV_SLAM_TRACKER_GRAB_IMAGE

    // Extractors
    const ORB::Extractor::Ptr mpExtractor0, mpExtractor1;

    explicit Tracker(System *pSystem, const YAML::Node &cfg
    ) : mpSystem(pSystem), mpIMU(IMU::Device::from_yaml(cfg["imu"])),
        mpCam0(camera::from_yaml(cfg["cam0"])), mpCam1(camera::from_yaml(cfg["cam1"])),
        mpExtractor0(ORB::Extractor::from_yaml(cfg["orb0"])),
        mpExtractor1(ORB::Extractor::from_yaml(cfg["orb1"])) {
      ASSERT(!is_monocular(), "Not implemented")
      ASSERT(mpCam0, "Camera0 not found")
      if (is_stereo()) {
        ASSERT(mpCam0->get_type() == mpCam1->get_type(), "Camera0 and Camera1 must be the same type")
        T_cam0_cam1 = mpCam0->T_cam_imu * mpCam1->T_cam_imu.inverse();
      }

      ASSERT(mpExtractor0, "Extractor0 not found")
      ASSERT((!mpCam1) == (!mpExtractor1), "miss match between Camera1 and Extractor1")
    }

    void run();
};

}

#endif
