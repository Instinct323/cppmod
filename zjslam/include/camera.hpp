#ifndef ZJSLAM__CAMERA_HPP
#define ZJSLAM__CAMERA_HPP

#include "camera/base.hpp"
#include "camera/calib.hpp"
#include "camera/kannala_brandt.hpp"
#include "camera/pinhole.hpp"
#include "file.hpp"

namespace camera {


Base* fromYAML(const YAML::Node &node) {
  // todo
}
}

#endif
