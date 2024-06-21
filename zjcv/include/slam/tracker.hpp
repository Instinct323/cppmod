#ifndef ZJCV__SLAM__TRACKER_HPP
#define ZJCV__SLAM__TRACKER_HPP

#include <yaml-cpp/yaml.h>

#include "camera.hpp"
#include "dataset/base.hpp"
#include "extension/orb.hpp"
#include "imu_type.hpp"

namespace slam {


class Tracker {

public:
    typedef std::shared_ptr<Tracker> Ptr;

    camera::Base::Ptr mpCam0, mpCam1;
    IMU::Device::Ptr mpIMU;
    ORB::Extractor::Ptr mpExtractor0, mpExtractor1;

    explicit Tracker(YAML::Node cfg) :
        mpCam0(camera::fromYAML(cfg["cam0"])), mpCam1(camera::fromYAML(cfg["cam1"])),
        mpIMU(IMU::Device::fromYAML(cfg["imu"])),
        mpExtractor0(ORB::Extractor::fromYAML(cfg["orb0"])), mpExtractor1(ORB::Extractor::fromYAML(cfg["orb1"])) {
      // 至少需要一个相机, 相机与特征提取器一一对应
      ASSERT(mpCam0 != nullptr && mpExtractor0 != nullptr, "Camera0 not found")
      ASSERT((mpCam1 == nullptr) == (mpExtractor1 == nullptr), "miss match between Camera1 and Extractor1")
    };

    void GrabMonocular(const double &timestamp, const cv::Mat &img0,
                       const dataset::IMUsamples &vImu = dataset::IMUsamples()) {
      ASSERT(mpCam1 == nullptr, "Wrong function for stereo camera")
      ASSERT((mpIMU == nullptr) == vImu.empty(), "miss match between IMU device and IMU samples")
      // todo
    }

    void GrabStereo(const double &timestamp, const cv::Mat &img0, const cv::Mat &img1,
                    const dataset::IMUsamples &vImu = dataset::IMUsamples()) {
      ASSERT(mpCam1 != nullptr, "Wrong function for monocular camera")
      ASSERT((mpIMU == nullptr) == vImu.empty(), "miss match between IMU device and IMU samples")
      // todo
    }
};

}

#endif
