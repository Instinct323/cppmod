#ifndef ZJCV__CAMERA_HPP
#define ZJCV__CAMERA_HPP

#include "camera/base.hpp"
#include "camera/kannala_brandt.hpp"
#include "camera/pinhole.hpp"
#include "utils/file.hpp"

namespace camera {

// YAML -> Camera::Ptr
Base::Ptr from_yaml(const YAML::Node &cfg);

// 通过棋盘格标定相机
void calib_by_chessboard(std::vector<std::string> &filenames, cv::Mat &distCoeffs,
                         cv::Size boardSize, cv::Size imgSize = {-1, -1}, int delay = 1);

}

#endif
