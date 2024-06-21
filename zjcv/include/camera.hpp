#ifndef ZJCV__CAMERA_HPP
#define ZJCV__CAMERA_HPP

#include "camera/base.hpp"
#include "camera/calib.hpp"
#include "camera/kannala_brandt.hpp"
#include "camera/pinhole.hpp"
#include "file.hpp"

namespace camera {


// YAML -> Camera::Ptr
Base::Ptr fromYAML(const YAML::Node &node) {
  Base::Ptr pCam;
  if (!node.IsNull()) {

    std::string type = node["type"].as<std::string>();
    auto imgSize = YAML::toVec<int>(node["resolution"]);
    auto intrinsics = YAML::toVec<float>(node["intrinsics"]);
    auto distCoeffs = YAML::toVec<float>(node["dist_coeffs"]);
    auto T_cam_imu = YAML::toSE3d(node["T_cam_imu"]);

    if (type == "Pinhole") {
      pCam = static_cast<Base::Ptr>(
          new Pinhole({imgSize[0], imgSize[1]}, intrinsics, distCoeffs, T_cam_imu)
      );
    } else if (type == "KannalaBrandt") {
      pCam = static_cast<Base::Ptr>(
          new KannalaBrandt({imgSize[0], imgSize[1]}, intrinsics, distCoeffs, T_cam_imu)
      );
    } else {
      LOG(FATAL) << "Invalid camera type: " << type;
    }
  }
  return pCam;
}

}

#endif
