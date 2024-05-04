#ifndef ZJSLAM__CAMERA_HPP
#define ZJSLAM__CAMERA_HPP

#include "camera/base.hpp"
#include "camera/kannala_brandt.hpp"
#include "camera/pinhole.hpp"
#include "file.hpp"

namespace camera {

// YAML -> Camera::Ptr
template<typename CameraT>
typename CameraT::Ptr fromYAML(const YAML::Node &node) {
  auto imgSize = YAML::toVec<int>(node["resolution"]);
  return typename CameraT::Ptr(new CameraT(
      {imgSize[0], imgSize[1]},
      YAML::toVec<float>(node["intrinsics"]),
      YAML::toVec<float>(node["dist_coeffs"]),
      YAML::toSE3d(node["T_cam_imu"])
  ));
}
}

#endif
