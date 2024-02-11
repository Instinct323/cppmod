#include "frame.h"
#include "frontend.h"


Frame::Ptr Frame::create(const cv::Mat &img, const Camera::Ptr &camera, const Frame::Ptr &last_frame) {
  if (last_frame == nullptr) {
    // 初始化第一帧
    Ptr p_init = Ptr(new Frame(img, camera));
    p_init->weak_this = p_init;
    return p_init;
  } else {
    // 尝试追踪上一帧
    Ptr p_track = Ptr(new Frame(img, camera, last_frame));
    p_track->weak_this = p_track;
    // 追踪状态差, 重新初始化
    if (p_track->status != FrameStatus::TRACK_GOOD) {
      Ptr p_init = Ptr(new Frame(img, camera));
      p_init->weak_this = p_init;
      p_init->link = p_track;
      return p_init;
    }
    return p_track;
  }
}


Frame::Frame(const cv::Mat &img,
             const Camera::Ptr &camera,
             const Ptr &last_frame
) : img(img), camera(camera), _Tcw(camera->Tcw) {
  if (last_frame != nullptr) {
    // 检测新特征点
    std::vector<cv::KeyPoint> org_kps;
    detector->detect(img, org_kps);
    for (auto &org_kp: org_kps) { kps.emplace_back(org_kp.pt); }
    // 更新状态
    kps.shrink_to_fit();
    status = (kps.size() >= nfeats_good) ? FrameStatus::INIT_SUCCESS : FrameStatus::INIT_FAILED;
  } else {
    // 尝试追踪上一帧
    std::vector<cv::Point2f> last_kps, cur_kps;
    for (auto kp: last_frame->kps) {
      last_kps.push_back(kp);
      cur_kps.push_back(kp);
    }
    // 光流匹配, 存储整理, 位姿求解
    match_keypoints(last_frame, last_kps, cur_kps);
    int nfeats = reduce(last_frame, cur_kps);
    // 状态更新
    status = (nfeats >= nfeats_good) ? FrameStatus::TRACK_GOOD : (
        (nfeats >= nfeats_bad) ? FrameStatus::TRACK_BAD : FrameStatus::TRACK_LOST);
  }
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
