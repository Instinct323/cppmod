#ifndef ZJCV__SLAM__OPTIMIZE_HPP
#define ZJCV__SLAM__OPTIMIZE_HPP

#include <cmath>
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
  const g2o::SE3Quat &T_cw = static_cast<VertexSE3Expmap *>(_vertices[0])->estimate(); \
  Eigen::Vector3d Pc = T_cw.map(Pw); \
  is_depth_positive = Pc[2] > 0; \
  if (is_depth_positive) { Pc[2] += 1e-2; } \
  else { Pc[2] -= 1e-2; }


namespace g2o {


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


namespace slam::feature {

bool optimize_pose(const std::shared_ptr<Frame> &pFrame, const std::shared_ptr<Frame> &pRefFrame, int n_iters = 3);


// 光束法平差
template<template<typename> class LinearSolverTp>
class BundleAdjustment : public g2o::Optimizer<6, 3, LinearSolverTp> {

public:
    typedef g2o::SparseOptimizer::Vertex Vertex;
    typedef g2o::SparseOptimizer::Edge Edge;
    typedef std::vector<std::shared_ptr<Frame>> FrameSharedPtrs;
    typedef std::vector<std::weak_ptr<Mappoint>> MappointWeakPtrs;

    bool only_pose;

    // Raw data
    FrameSharedPtrs::iterator iFramesBeg, iFramesEnd;
    MappointWeakPtrs::iterator iMapptsBeg, iMapptsEnd;
    size_t n_frames = 0, n_mappts = 0, id_vex = 0;

    // Information, Robust kernel
    Eigen::Matrix2d info2 = 3 * Eigen::Matrix2d::Identity();
    Eigen::Matrix3d info3 = 2 * Eigen::Matrix3d::Identity();
    g2o::RobustKernelHuber *rk = nullptr;

    // Processed data
    std::vector<std::vector<Edge *>> edgeMappt;
    std::vector<size_t> badMappt;

    BundleAdjustment(size_t idRefFrame, bool only_pose,
                     FrameSharedPtrs::iterator iFramesBeg, FrameSharedPtrs::iterator iFramesEnd,
                     MappointWeakPtrs::iterator iMapptsBeg, MappointWeakPtrs::iterator iMapptsEnd
    ) : g2o::Optimizer<6, 3, LinearSolverTp>(), only_pose(only_pose),
        iFramesBeg(iFramesBeg), iFramesEnd(iFramesEnd), iMapptsBeg(iMapptsBeg), iMapptsEnd(iMapptsEnd),
        n_frames(iFramesEnd - iFramesBeg), n_mappts(iMapptsEnd - iMapptsBeg), id_vex(n_frames + n_mappts),
        edgeMappt(n_mappts) {
      // Vertex: 帧位姿
      for (size_t i = 0; i < n_frames; ++i) {
        Frame::Ptr &pF = *(iFramesBeg + i);
        auto vFrame = add_frame(i, pF);
        vFrame->setFixed(pF->mId == idRefFrame);
        this->addVertex(vFrame);
      }
      for (size_t i = 0; i < n_mappts; ++i) {
        auto it = iMapptsBeg + i;
        if (it->expired()) {
          badMappt.push_back(i);
          continue;
        }
        auto pMappt = it->lock();
        // Vertex: 地图点
        Vertex *vMappt = add_mappt(n_frames + i);
        if (vMappt) this->addVertex(vMappt);
        // Edge: 地图点投影
        Eigen::Vector3d Pw = pMappt->mPos.cast<double>();
        {
          parallel::ScopedLock lock(pMappt->apObs.mutex);
          edgeMappt[i].reserve(pMappt->apObs->size());
          for (auto obs: *pMappt->apObs) {
            Edge *eObs = add_observation(i, Pw, obs, vMappt);
            if (eObs) edgeMappt[i].push_back(eObs);
          }
        }
        // 离群点标记
        if (edgeMappt[i].size() < 2) {
          badMappt.push_back(i);
          edgeMappt[i].clear();
        } else {
          for (auto e: edgeMappt[i]) this->addEdge(e);
        }
      }
    }

    void reset() {
      // Vertex: 帧位姿
      for (size_t i = 0; i < n_frames; ++i) {
        Frame::Ptr &pFrame = *(iFramesBeg + i);
        auto vFrame = static_cast<g2o::VertexSE3Expmap *>(this->vertex(i));
        vFrame->setEstimate(Sophus::toG2O(pFrame->mPose.T_imu_world));
      }
      // Vertex: 地图点
      if (!only_pose) {
        for (size_t i = 0; i < n_mappts; ++i) {
          auto it = iMapptsBeg + i;
          if (it->expired()) continue;
          auto pMappt = it->lock();
          auto vMappt = static_cast<g2o::VertexPointXYZ *>(this->vertex(n_frames + i));
          vMappt->setEstimate(pMappt->mPos.cast<double>());
        }
      }
    }

    Vertex *add_frame(size_t id, Frame::Ptr &pFrame) {
      pFrame->mIdVex = id;
      auto *vFrame = new g2o::VertexSE3Expmap;
      vFrame->setId(id);
      return static_cast<Vertex *>(vFrame);
    }

    Vertex *add_mappt(size_t id) {
      if (only_pose) return nullptr;
      auto vMappt = new g2o::VertexPointXYZ;
      vMappt->setId(id);
      vMappt->setFixed(false);
      return static_cast<Vertex *>(vMappt);
    }

    Edge *add_observation(size_t mp_id, const Eigen::Vector3d &Pw, Observation &obs, Vertex *vMappt) {
      auto [wpf, idx] = obs;
      if (wpf.expired()) return nullptr;
      auto pFrame = wpf.lock();
      Edge *eObs;
      Eigen::Vector3d Pc = pFrame->mvUnprojs0[idx].cast<double>();
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
      // 设置顶点
      auto vFrame = this->vertex(pFrame->mIdVex);
      if (!vFrame) {
        vFrame = add_frame(id_vex, pFrame);
        vFrame->setFixed(true);
        this->addVertex(vFrame);
        id_vex++;
      }
      eObs->setId(pFrame->mIdVex * n_mappts + mp_id);
      eObs->setVertex(0, vFrame);
      if (!only_pose) eObs->setVertex(1, static_cast<g2o::VertexPointXYZ *>(vMappt));
      return eObs;
    }

    size_t valid_mappts() { return n_mappts - badMappt.size(); }

    size_t outlier_rejection(float thresh = 5.991) {
      size_t n = 0, m = 0;
      long double sumvar = 0;
      // 假设误差项的均值为 0, 累计方差
      for (size_t i = 0; i < n_mappts; i++) {
        if (edgeMappt[i].empty()) continue;
        for (auto e: edgeMappt[i]) {
          double chi2 = e->chi2();
          n++;
          if (!std::isfinite(chi2)) continue;
          sumvar += chi2;
        }
      }
      // 根据方差计算阈值
      float th = thresh * float(sumvar / (long double) n);
      for (size_t i = 0; i < n_mappts; i++) {
        if (edgeMappt[i].empty()) continue;
        auto it = edgeMappt[i].begin();
        while (it != edgeMappt[i].end()) {
          double chi2 = (*it)->chi2();
          if (chi2 < thresh) {
            ++it;
          } else {
            // 离群点筛除
            m++;
            (*it)->setLevel(1);
            edgeMappt[i].erase(it);
          }
        }
        // 离群地图点标记
        if (edgeMappt[i].size() < 2) {
          badMappt.push_back(i);
          edgeMappt[i].clear();
        }
      }
      return m;
    }

    void apply_result(bool clear_mp) {
      // Frame: 位姿
      for (size_t i = 0; i < n_frames; ++i) {
        auto v = static_cast<g2o::VertexSE3Expmap *>(this->vertex(i));
        if (v) (*(iFramesBeg + i))->mPose.set_pose(g2o::toSE3(v->estimate()));
      }
      // Mappoint: 离群点
      for (size_t i: badMappt) {
        if ((iMapptsBeg + i)->expired()) continue;
        auto pMappt = (iMapptsBeg + i)->lock();
        if (clear_mp) {
          pMappt->clear();
        } else {
          pMappt->set_invalid();
        }
      }
      // Mappoint: 位置
      if (!only_pose) {
        for (size_t i = 0; i < n_mappts; ++i) {
          if ((iMapptsBeg + i)->expired()) continue;
          auto pMappt = (iMapptsBeg + i)->lock();
          auto v = static_cast<g2o::VertexPointXYZ *>(this->vertex(n_frames + i));
          if (v) pMappt->set_pos(v->estimate().cast<float>());
        }
      }
    }
};

}

#endif
