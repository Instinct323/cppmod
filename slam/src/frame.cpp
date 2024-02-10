#include "frame.h"
#include "frontend.h"


Frame::Frame(const Camera::Ptr &camera,
                      const cv::Mat &img,
                      const Ptr &last_frame,
                      const SE3 &T01,   // 相机运动 (lastWorld <- curWorld)
                      const cv::Ptr<cv::Feature2D> &detector
) : camera(camera), img(img) {

  std::vector<cv::Point2f> last_kps, cur_kps;

  SE3 Tc0 = camera->Tcw;  // camera <- lastWorld
  // 暂时修改相机位姿
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
  is_init = match_keypoints(last_frame, last_kps, cur_kps) < Frontend::nfeats_track_good && detector != nullptr;
  if (is_init) {
    // 匹配到的特征点不足, 检测新特征点
    std::vector<cv::KeyPoint> org_kps;
    detector->detect(img, org_kps);
    for (auto &org_kp: org_kps) { kps.emplace_back(org_kp.pt); }
  } else {
    // 转化存储: cur_kps -> kps
    for (int i = 0; i < cur_kps.size(); i++) {
      kps.emplace_back(cur_kps[i], last_frame->kps[i]._mappoint);
    }
    // todo: 上一帧是初始化状态, 创建路标点

    reduce();
  }
}
