#ifndef UTILS__RS_HPP
#define UTILS__RS_HPP

#include <librealsense2/rs.hpp>
#include <map>
#include <opencv4/opencv2/opencv.hpp>
#include <sophus/se3.hpp>

namespace rs2 {

void devices_profile();

void frames_profile(const rs2::frameset &frames);

// rs2::frame -> cv::Mat
extern std::map<rs2_format, int> cv_fmt_lut;

cv::Mat toCvMat(const rs2::video_frame &vf);

// rs2::frame -> Sophus
Sophus::SE3f toSE3(const rs2::pose_frame &pf);

// rs2::frame -> Vector3f
Eigen::Vector3f toVec3f(const rs2::motion_frame &mf);

}

#endif
