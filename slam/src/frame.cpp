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
    link = (last_frame->link == nullptr) ? last_frame : last_frame->link;
    std::vector<cv::Point2f> last_kps, cur_kps;
    for (Keypoint kp: last_frame->kps) {
      last_kps.push_back(kp);
      cur_kps.push_back(kp);
    }
    // 光流匹配, 存储整理
    match_keypoints(last_frame, last_kps, cur_kps);
    int nfeats = reduce(last_frame, cur_kps);
    // 状态更新
    status = (nfeats >= nfeats_min) ? TRACK_GOOD : TRACK_LOST;
    if (status == TRACK_GOOD) {
      float nfeats_thresh = nfeats_decay * static_cast<float>(link->kps.size());
      if (nfeats < nfeats_thresh) status = TRACK_BAD;
    }
  }
}


int Frame::reduce(const Ptr &last_frame,
                  std::vector<cv::Point2f> &cur_kps) {
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
      if (mp != nullptr && mp->is_inlier) {
        mp->add(weak_this, kps.size());
        mp->triangulation();
        kps.emplace_back(cur_kps[i], mp);
      }
    }
  }
  // 清空状态
  kp_status.clear();
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
    optimizer.addVertex(vex_pose);

    for (int i = 0; i < kps.size(); i++) {
      Keypoint kp = kps[i];
      Mappoint::Ptr mp = kp.mp;
      if (mp == nullptr || !mp->is_inlier) continue;
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

    for (int i = 0; i < 5; i++) {
      vex_pose->setEstimate(Tcw);
      optimizer.setVerbose(true);
      optimizer.initializeOptimization();
      optimizer.optimize(10);

      for (auto &edge: edges) {
        if (!edge->mp->is_inlier) continue;
        edge->computeError();
        // 根据 chi2 阈值设置边、路标点的 inlier 状态
        edge->mp->is_inlier = edge->chi2() < chi2_th;
        edge->setLevel(!edge->mp->is_inlier);
        // if (i == 2) edge->setRobustKernel(nullptr);
      }

      Tcw = vex_pose->estimate();
    }
  }
}
