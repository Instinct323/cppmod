#include "frame.h"
#include "frontend.h"


Frame::Frame(const Camera::Ptr &camera,
             const cv::Mat &img,
             const Ptr &last_frame,
             const SE3 &T01,   // 相机运动 (lastWorld <- curWorld)
             const cv::Ptr<cv::Feature2D> &detector
) : camera(camera), img(img) {
  std::vector<cv::Point2f> last_kps, cur_kps;
  // 暂时修改相机位姿
  SE3 Tc0 = camera->Tcw;  // camera <- lastWorld
  camera->set_Tcw(Tc0 * T01);
  for (auto kp: last_frame->kps) {
    last_kps.push_back(kp);
    // 使用 last_kps 初始化 cur_kps
    auto mp = kp.getMappoint();
    if (mp != nullptr) kp = camera->world2pixel(mp->p_w);
    cur_kps.push_back(kp);
  }
  // 还原相机位姿
  camera->set_Tcw(Tc0);
  // 光流匹配关键点
  int nfeats = match_keypoints(last_frame, last_kps, cur_kps);
  switch_status(nfeats, last_frame, detector);
  reduce(last_frame, cur_kps);
}


void Frame::set_pose(const SE3 *pose) {
  if (pose != nullptr) {
    // 给定位姿时, 直接设置
    _Tcw = *pose;
    has_Tcw = true;
  } else {
    // 未给定位姿时, 通过 g2o 优化位姿

  }
}
