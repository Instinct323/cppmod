#ifndef ZJCV__CAMERA_HPP
#define ZJCV__CAMERA_HPP

#include "camera/base.hpp"
#include "camera/kannala_brandt.hpp"
#include "camera/pinhole.hpp"
#include "file.hpp"

namespace camera {

// YAML -> Camera::Ptr
Base::Ptr fromYAML(const YAML::Node &node);

void calibByChessboard(std::vector<std::string> &filenames, cv::Mat distCoeffs,
                       cv::Size boardSize, cv::Size imgSize = {-1, -1}, int delay = 1);

}

#endif
