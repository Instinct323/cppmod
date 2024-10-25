#include "utils/rs2.hpp"

namespace rs2 {


void devices_profile() {
  rs2::context ctx;
  for (int i = 0; i < ctx.query_devices().size(); i++) {
    std::cout << "Device " << i << ": " << ctx.query_devices()[i].get_info(RS2_CAMERA_INFO_NAME) << std::endl;
  }
}


void frames_profile(const rs2::frameset &frames) {
  frames.foreach_rs([&](rs2::frame frame) {
    rs2::stream_profile profile = frame.get_profile();
    std::cout << profile.stream_type() << ": " << profile.format() << std::endl;
  });
}


std::map<rs2_format, int> cv_fmt_lut = {
    {RS2_FORMAT_Y8,   CV_8UC1},
    {RS2_FORMAT_BGR8, CV_8UC3},
    {RS2_FORMAT_RGB8, CV_8UC3},
    {RS2_FORMAT_Z16,  CV_16UC1},
};


// rs2::frame -> cv::Mat
cv::Mat toCvMat(const rs2::video_frame &vf) {
  cv::Size size(vf.get_width(), vf.get_height());
  int cvt = cv_fmt_lut[vf.get_profile().format()];
  CV_Assert(cvt);
  return {size, cvt, (void *)vf.get_data(), cv::Mat::AUTO_STEP};
}


// rs2::frame -> Sophus
Sophus::SE3f toSE3(const rs2::pose_frame &pf) {
  auto pose = pf.get_pose_data();
  Eigen::Quaternionf q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
  Eigen::Vector3f t(pose.translation.x, pose.translation.y, pose.translation.z);
  return {q, t};
}


// rs2::frame -> Vector3f
Eigen::Vector3f toVec3f(const rs2::motion_frame &mf) {
  auto data = mf.get_motion_data();
  return {data.x, data.y, data.z};
}

}
