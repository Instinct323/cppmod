#include "frame.h"
#include "g2o_types.h"


Frame::Frame(const cv::Mat &img,
             const Camera::Ptr &camera,
             const Ptr &last_frame
) : img(img), camera(camera), Tcw(camera->Tcw) {
  if (last_frame != nullptr) {
    // 检测新特征点
    std::vector<cv::KeyPoint> org_kps;
    detector->detect(img, org_kps);
    for (cv::KeyPoint &org_kp: org_kps) { kps.emplace_back(org_kp.pt); }
    // 更新状态
    kps.shrink_to_fit();
    status = (kps.size() >= nfeats_min * 2) ? INIT_SUCCESS : INIT_FAILED;
  } else {
    // 尝试追踪上一帧
    link = (last_frame->status == INIT_SUCCESS) ? last_frame : last_frame->link;
    std::vector<cv::Point2f> last_kps, cur_kps;
    for (Keypoint kp: last_frame->kps) {
      last_kps.push_back(kp);
      cur_kps.push_back(kp);
    }
    // 光流匹配, 存储整理
    int nfeats = match_keypoints(last_frame, last_kps, cur_kps);
    // 状态更新
    status = (nfeats >= nfeats_min) ? TRACK_GOOD : TRACK_LOST;
    if (status == TRACK_GOOD) {
      float nfeats_thresh = nfeats_decay * static_cast<float>(link->kps.size());
      if (nfeats < nfeats_thresh) status = TRACK_BAD;
    }
  }
}


int Frame::match_keypoints(const Ptr &last_frame,
                           std::vector<cv::Point2f> &last_kps,
                           std::vector<cv::Point2f> &cur_kps) {
  std::vector<uchar> kp_status;
  cv::calcOpticalFlowPyrLK(
      last_frame->img, img, last_kps, cur_kps,
      kp_status, cv::Mat(), cv::Size(11, 11), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
      cv::OPTFLOW_USE_INITIAL_FLOW);

  for (int i = 0; i < kp_status.size(); i++) {
    // 转化存储: cur_kps -> kps (删除匹配失败的关键点, 过期的路标点)
    Mappoint::Ptr mp = last_frame->kps[i].mp;
    if (kp_status[i] != 0 && mp != nullptr) {
      // 上一帧是初始化状态, 创建路标点
      if (last_frame->status == INIT_SUCCESS) {
        mp = Mappoint::create();
        mp->add(last_frame, i);
      }
      // 更新路标点的关键点
      if (mp != nullptr) {
        mp->add(weak_this, kps.size());
        kps.emplace_back(cur_kps[i], mp);
      }
    }
  }
  kps.shrink_to_fit();
  return kps.size();
}


void Frame::mul_Tcw(const SE3 &motion, bool optimize, double chi2_th) {
  Tcw = motion * Tcw;
  has_Tcw = true;
  // g2o 优化位姿
  if (optimize) {
    Optimizer<6, 2, g2o::LinearSolverDense, g2o::OptimizationAlgorithmLevenberg> optimizer;
    std::vector<EdgePose *> edges;

    auto *vex_pose = new VertexPose;
    vex_pose->setId(0);
    vex_pose->setEstimate(Tcw);
    optimizer.addVertex(vex_pose);

    for (int i = 0; i < kps.size(); i++) {
      Keypoint kp = kps[i];
      Mappoint::Ptr mp = kp.mp;
      if (mp == nullptr) continue;
      // 估计值: 路标点世界坐标经位姿信息, 变换到像素坐标
      auto *edge = new EdgePose(mp, camera);
      edge->setId(i);
      edge->setVertex(0, vex_pose);
      edge->setMeasurement(kp);
      edge->setInformation(Mat22::Identity());
      edge->setRobustKernel(new g2o::RobustKernelHuber);
      edges.push_back(edge);
      optimizer.addEdge(edge);
    }

    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(40);
    Tcw = vex_pose->estimate();

    for (auto &edge: edges) {
      edge->computeError();
      // 根据 chi2 阈值设置路标点的 inlier 状态
      edge->mp->is_inlier = edge->chi2() < chi2_th;
    }
  }
}
