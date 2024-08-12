#ifndef ZJCV__SLAM__OPTIMIZE_HPP
#define ZJCV__SLAM__OPTIMIZE_HPP

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>

#include "frame.hpp"
#include "utils/g2o.hpp"


#define G2O_WITH_PWORLD \
  Eigen::Vector3d Pw; \
  void set_pworld(const Eigen::Vector3d &pw) { Pw = pw; }


#define G2O_EDGE_PROJECT_UTILS \
  bool is_depth_positive;


#define G2O_EDGE_PROJECT_MAP \
  const g2o::SE3Quat &T_wc = static_cast<VertexSE3Expmap *>(_vertices[0])->estimate(); \
  Eigen::Vector3d Pc = T_wc.map(Pw); \
  static_cast<ProjectEdgeData *>(this->userData())->is_depth_positive = Pc[2] > 0; \
  if (is_depth_positive) { Pc[2] += 1e-2; } \
  else { Pc[2] -= 1e-2; }


namespace g2o {


class ProjectEdgeData : public UserDataType {

public:
    G2O_EMPTY_SERIALIZE

    bool is_depth_positive = true;
};


class EdgeSE3ProjectDNP : public BaseBinaryEdge<2, Eigen::Vector2d, VertexSE3Expmap, VertexPointXYZ> {

public:
    G2O_EMPTY_SERIALIZE

    G2O_EDGE_PROJECT_UTILS

    void computeError() override {
      const Eigen::Vector3d &Pw = static_cast<VertexPointXYZ *>(_vertices[1])->estimate();
      G2O_EDGE_PROJECT_MAP
      _error = _measurement - Pc.head<2>() / Pc[2];
    }
};


class EdgeSE3ProjectDNPonlyPose : public BaseUnaryEdge<2, Eigen::Vector2d, VertexSE3Expmap> {

public:
    G2O_EMPTY_SERIALIZE

    G2O_WITH_PWORLD

    G2O_EDGE_PROJECT_UTILS

    void computeError() override {
      G2O_EDGE_PROJECT_MAP
      _error = _measurement - Pc.head<2>() / Pc[2];
    }
};


class EdgeStereoSE3ProjectDNP : public BaseBinaryEdge<3, Eigen::Vector3d, VertexSE3Expmap, VertexPointXYZ> {

public:
    G2O_EMPTY_SERIALIZE

    G2O_EDGE_PROJECT_UTILS

    void computeError() override {
      const Eigen::Vector3d &Pw = static_cast<VertexPointXYZ *>(_vertices[1])->estimate();
      G2O_EDGE_PROJECT_MAP
      _error = _measurement - Eigen::Vector3d(Pc[0] / Pc[2], Pc[1] / Pc[2], Pc[2]);
    }
};


class EdgeStereoSE3ProjectDNPonlyPose : public BaseUnaryEdge<3, Eigen::Vector3d, VertexSE3Expmap> {

public:
    G2O_EMPTY_SERIALIZE

    G2O_WITH_PWORLD

    G2O_EDGE_PROJECT_UTILS

    void computeError() override {
      G2O_EDGE_PROJECT_MAP
      _error = _measurement - Eigen::Vector3d(Pc[0] / Pc[2], Pc[1] / Pc[2], Pc[2]);
    }
};


}

#define BA_DEPTH_WEIGHT 1.

namespace slam::feature {

bool optimize_pose(System *pSystem, const std::shared_ptr<Frame> &pFrame, const std::shared_ptr<Frame> &pRefFrame, bool only_pose);


// 光束法平差
template<template<typename> class LinearSolverTp>
class BundleAdjustment : public g2o::Optimizer<6, 3, LinearSolverTp> {

public:
    typedef g2o::SparseOptimizer::Vertex Vertex;
    typedef g2o::SparseOptimizer::Edge Edge;
    typedef std::vector<std::shared_ptr<Frame>> FrameSharedPtrs;

    static std::mutex mtxIdVex;

    const Sophus::SE3f T_cam0_imu, T_imu_cam0;
    bool only_pose;

    // Raw data
    size_t n_frames;
    FrameSharedPtrs::iterator iFramesBeg, iFramesEnd;

    // Information, Robust kernel
    const Eigen::Vector4f thresh = {0, 3.841, 5.991, 7.815};
    Eigen::Matrix2d info2 = Eigen::Matrix2d::Identity();
    Eigen::Matrix3d info3 = Eigen::Matrix3d::Identity();
    g2o::RobustKernelHuber *rk = nullptr;

    // Processed data
    std::vector<std::shared_ptr<Mappoint>> vMappts;
    std::vector<std::vector<Edge *>> edgeMappts;

    explicit BundleAdjustment(const Sophus::SE3f &T_cam0_imu, size_t idRefFrame, bool only_pose,
                              FrameSharedPtrs::iterator iFramesBeg, FrameSharedPtrs::iterator iFramesEnd
    ) : g2o::Optimizer<6, 3, LinearSolverTp>(),
        T_cam0_imu(T_cam0_imu), T_imu_cam0(T_cam0_imu.inverse()), only_pose(only_pose),
        iFramesBeg(iFramesBeg), iFramesEnd(iFramesEnd), n_frames(iFramesEnd - iFramesBeg) {
      assert(n_frames > 0 && "Empty frames or map points");
      info3(2, 2) = BA_DEPTH_WEIGHT;
      parallel::ScopedLock lock(mtxIdVex);
      // Vertex: 帧位姿
      for (size_t i = 0; i < n_frames; ++i) {
        Frame::Ptr &pF = *(iFramesBeg + i);
        Vertex *vFrame = add_frame(i);
        vFrame->setFixed(pF->mId == idRefFrame);
        for (auto obs: pF->mmpMappts) {
          // Edge: 地图点投影
          add_observation(pF, obs);
        }
      }
      // 还原 mIdVex
      for (size_t i = 0; i < n_frames; ++i) (*(iFramesBeg + i))->mIdVex = SIZE_MAX;
      for (auto &pMappt: vMappts) pMappt->mIdVex = SIZE_MAX;
    }

    void reset() {
      // Vertex: 帧位姿
      for (size_t i = 0; i < n_frames; ++i) {
        Frame::Ptr &pFrame = *(iFramesBeg + i);
        auto vFrame = static_cast<g2o::VertexSE3Expmap *>(this->vertex(i));
        vFrame->setEstimate(Sophus::toG2O(T_imu_cam0 * pFrame->mPose.T_world_imu));
      }
      // Vertex: 地图点
      if (!only_pose) {
        for (size_t i = 0; i < vMappts.size(); ++i) {
          auto vMappt = static_cast<g2o::VertexPointXYZ *>(this->vertex(n_frames + i));
          vMappt->setEstimate(vMappts[i]->mPos.cast<double>());
        }
      }
    }

    bool is_bad_mappts(size_t i) {
      // 只有一个观测, 而且是 EdgeSE3ProjectDNP
      return edgeMappts[i].empty() || (!only_pose && edgeMappts[i].size() == 1 && edgeMappts[i][0]->dimension() == 2);
    }

    size_t outlier_rejection(bool detect_depth_positive = false) {
      size_t n = 0, m = 0;
      long double sumvar = 0;
      // 假设误差项的均值为 0, 累计方差
      for (size_t i = 0; i < edgeMappts.size(); i++) {
        if (edgeMappts[i].empty()) continue;
        for (auto e: edgeMappts[i]) {
          if (e->level()) continue;
          double chi2 = e->chi2();
          if (!std::isfinite(chi2) || chi2 > 1e9) continue;
          n++;
          sumvar += chi2;
        }
      }
      // 根据方差计算阈值
      Eigen::Vector4f th = thresh * float(sumvar / (long double) n);
      for (size_t i = 0; i < edgeMappts.size(); i++) {
        if (edgeMappts[i].empty()) continue;
        auto it = edgeMappts[i].begin();
        while (it != edgeMappts[i].end()) {
          if ((*it)->level()) continue;
          int d = (*it)->dimension();
          double chi2 = (*it)->chi2();
          if (chi2 < th[d] && (!detect_depth_positive ||
                               static_cast<g2o::ProjectEdgeData *>((*it)->userData())->is_depth_positive)) {
            ++it;
          } else {
            // 离群点筛除
            m++;
            (*it)->setLevel(1);
            edgeMappts[i].erase(it);
            int edge_id = (*it)->id(), mappt_id = edge_id / n_frames, frame_id = edge_id % n_frames;
            vMappts[mappt_id]->erase_obs(*(iFramesBeg + frame_id));
          }
        }
        // 有效观测不足
        if (is_bad_mappts(i)) {
          for (auto e: edgeMappts[i]) e->setLevel(1);
          edgeMappts[i].clear();
        }
      }
      return m;
    }

    void print_edge() {
      LOG(INFO) << "----- Edges -----";
      for (auto *e_: this->edges()) {
        auto e = static_cast<Edge *>(e_);
        if (e->level()) continue;
        if (e->dimension() == 2) {
          auto ed = static_cast<g2o::EdgeSE3ProjectDNPonlyPose *>(e);
          LOG(INFO) << ed->measurement().transpose() << " -> " << ed->error().transpose();
        } else {
          auto ed = static_cast<g2o::EdgeStereoSE3ProjectDNPonlyPose *>(e);
          LOG(INFO) << ed->measurement().transpose() << " -> " << ed->error().transpose();
        }
      }
      LOG(INFO) << "----- Edges -----";
    }

    void apply_result() {
      // Frame: 位姿
      for (size_t i = 0; i < n_frames; ++i) {
        auto v = static_cast<g2o::VertexSE3Expmap *>(this->vertex(i));
        if (v) (*(iFramesBeg + i))->mPose.set_pose(T_cam0_imu * g2o::toSE3(v->estimate()));
      }
      // Mappoint: 位置
      if (!only_pose) {
        for (size_t i = 0; i < vMappts.size(); ++i) {
          auto pMappt = vMappts[i];
          if (is_bad_mappts(i)) pMappt->set_invalid();
          auto v = static_cast<g2o::VertexPointXYZ *>(this->vertex(n_frames + i));
          if (v) pMappt->set_pos(v->estimate().cast<float>());
        }
      }
    }

protected:

    Vertex *add_frame(size_t id) {
      Frame::Ptr &pFrame = *(iFramesBeg + id);
      Vertex *vFrame;
      if (pFrame->mIdVex >= n_frames) {
        // 新增帧位姿
        pFrame->mIdVex = id;
        vFrame = static_cast<Vertex *>(new g2o::VertexSE3Expmap);
        vFrame->setId(id);
        this->addVertex(vFrame);
      } else {
        vFrame = this->vertex(pFrame->mIdVex);
      }
      return vFrame;
    }

    Vertex *add_mappt(Mappoint::Ptr &pMappt) {
      Vertex *vMappt;
      if (pMappt->mIdVex >= vMappts.size()) {
        // 新增地图点
        pMappt->mIdVex = vMappts.size();
        vMappts.push_back(pMappt);
        edgeMappts.emplace_back();
        // Vertex: 地图点
        if (!only_pose) {
          vMappt = static_cast<Vertex *>(new g2o::VertexPointXYZ);
          vMappt->setId(n_frames + pMappt->mIdVex);
          vMappt->setFixed(false);
          this->addVertex(vMappt);
        }
      } else if (!only_pose) {
        vMappt = this->vertex(n_frames + pMappt->mIdVex);
      }
      return vMappt;
    }

    Edge *add_observation(Frame::Ptr pFrame, std::pair<int, Mappoint::Ptr> obs) {
      Mappoint::Ptr &pMappt = obs.second;
      Edge *eObs;
      // Vertex: 帧位姿, 地图点
      Vertex *vFrame = this->vertex(pFrame->mIdVex), *vMappt = add_mappt(pMappt);
      assert(vFrame);
      Eigen::Vector3d Pw = pMappt->mPos.cast<double>(),
          Pc = pFrame->mvUnprojs0[obs.first].cast<double>();
      // 双目标记, 双目观测的 z 为实际值的相反数
      if (Pc[2] < 0) {
        Pc[2] = -Pc[2];
        if (only_pose) {
          auto eTmp = new g2o::EdgeStereoSE3ProjectDNPonlyPose;
          eTmp->set_pworld(Pw);
          eTmp->setMeasurement(Pc);
          eTmp->setInformation(info3);
          eObs = static_cast<Edge *>(eTmp);
        } else {
          auto eTmp = new g2o::EdgeStereoSE3ProjectDNP;
          eTmp->setMeasurement(Pc);
          eTmp->setInformation(info3);
          eObs = static_cast<Edge *>(eTmp);
        }
      } else {
        if (only_pose) {
          auto eTmp = new g2o::EdgeSE3ProjectDNPonlyPose;
          eTmp->set_pworld(Pw);
          eTmp->setMeasurement(Pc.head<2>());
          eTmp->setInformation(info2);
          eObs = static_cast<Edge *>(eTmp);
        } else {
          auto eTmp = new g2o::EdgeSE3ProjectDNP;
          eTmp->setMeasurement(Pc.head<2>());
          eTmp->setInformation(info2);
          eObs = static_cast<Edge *>(eTmp);
        }
      }
      eObs->setId(pMappt->mIdVex * n_frames + pFrame->mIdVex);
      eObs->setUserData(new g2o::ProjectEdgeData);
      edgeMappts[pMappt->mIdVex].push_back(eObs);
      // 设置顶点
      eObs->setVertex(0, vFrame);
      if (!only_pose) eObs->setVertex(1, static_cast<g2o::VertexPointXYZ *>(vMappt));
      assert(this->addEdge(eObs));
      return eObs;
    }
};

}

#endif
