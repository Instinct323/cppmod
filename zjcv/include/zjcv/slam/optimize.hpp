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
    typedef std::vector<std::weak_ptr<Mappoint>> MappointWeakPtrs;

    const Sophus::SE3f T_cam0_imu, T_imu_cam0;
    bool only_pose;

    // Raw data
    FrameSharedPtrs::iterator iFramesBeg, iFramesEnd;
    MappointWeakPtrs::iterator iMapptsBeg, iMapptsEnd;
    size_t n_frames = 0, n_mappts = 0;

    // Information, Robust kernel
    const Eigen::Vector4f thresh = {0, 3.841, 5.991, 7.815};
    Eigen::Matrix2d info2 = Eigen::Matrix2d::Identity();
    Eigen::Matrix3d info3 = Eigen::Matrix3d::Identity();
    g2o::RobustKernelHuber *rk = nullptr;

    // Processed data
    std::vector<std::vector<Edge *>> edgeMappts;

    BundleAdjustment(const Sophus::SE3f &T_cam0_imu, size_t idRefFrame, bool only_pose,
                     FrameSharedPtrs::iterator iFramesBeg, FrameSharedPtrs::iterator iFramesEnd,
                     MappointWeakPtrs::iterator iMapptsBeg, MappointWeakPtrs::iterator iMapptsEnd
    ) : g2o::Optimizer<6, 3, LinearSolverTp>(),
        T_cam0_imu(T_cam0_imu), T_imu_cam0(T_cam0_imu.inverse()), only_pose(only_pose),
        iFramesBeg(iFramesBeg), iFramesEnd(iFramesEnd), iMapptsBeg(iMapptsBeg), iMapptsEnd(iMapptsEnd),
        n_frames(iFramesEnd - iFramesBeg), n_mappts(iMapptsEnd - iMapptsBeg),
        edgeMappts(n_mappts) {
      ASSERT(n_frames > 0 && n_mappts > 0, "Empty frames or map points")
      info3(2, 2) = BA_DEPTH_WEIGHT;
      // Vertex: 帧位姿
      for (size_t i = 0; i < n_frames; ++i) {
        Frame::Ptr &pF = *(iFramesBeg + i);
        auto vFrame = add_frame(i, pF);
        vFrame->setFixed(pF->mId == idRefFrame);
        this->addVertex(vFrame);
      }
      for (size_t i = 0; i < n_mappts; ++i) {
        auto it = iMapptsBeg + i;
        if (it->expired()) continue;
        auto pMappt = it->lock();
        if (only_pose && pMappt->is_invalid()) continue;
        // Vertex: 地图点
        Vertex *vMappt = add_mappt(n_frames + i);
        if (vMappt) this->addVertex(vMappt);
        // Edge: 地图点投影
        Eigen::Vector3d Pw = pMappt->mPos.cast<double>();
        {
          parallel::ScopedLock lock(pMappt->apObs.mutex);
          edgeMappts[i].reserve(pMappt->apObs->size());
          for (auto obs: *pMappt->apObs) {
            Edge *eObs = add_observation(i, Pw, obs, vMappt);
            if (!eObs) continue;
            edgeMappts[i].push_back(eObs);
          }
        }
        // 只有一个观测, 而且不是双目观测
        if (is_bad_mappts(i)) {
          edgeMappts[i].clear();
        } else {
          for (auto e: edgeMappts[i]) this->addEdge(e);
        }
      }
      // 还原 mIdVex
      for (size_t i = 0; i < n_frames; ++i) (*(iFramesBeg + i))->mIdVex = SIZE_MAX;
    }

    void reset() {
      // Vertex: 帧位姿
      for (size_t i = 0; i < n_frames; ++i) {
        Frame::Ptr &pFrame = *(iFramesBeg + i);
        auto vFrame = static_cast<g2o::VertexSE3Expmap *>(this->vertex(i));
        vFrame->setEstimate(Sophus::toG2O(T_cam0_imu * pFrame->mPose.T_imu_world));
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

    bool is_bad_mappts(size_t i) {
      // 只有一个观测, 而且是 EdgeSE3ProjectDNP
      return edgeMappts[i].empty() || (!only_pose && edgeMappts[i].size() == 1 && edgeMappts[i][0]->dimension() == 2);
    }

    size_t outlier_rejection(bool clean_obs, bool detect_depth_positive = false) {
      size_t n = 0, m = 0;
      long double sumvar = 0;
      // 假设误差项的均值为 0, 累计方差
      for (size_t i = 0; i < n_mappts; i++) {
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
      for (size_t i = 0; i < n_mappts; i++) {
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
            this->removeEdge(*it);
            edgeMappts[i].erase(it);
            if (clean_obs) {
              int edge_id = (*it)->id(), frame_id = edge_id / n_mappts, mappt_id = edge_id % n_mappts;
              auto itMappt = iMapptsBeg + mappt_id;
              if (itMappt->expired()) continue;
              itMappt->lock()->erase_obs(*(iFramesBeg + frame_id));
            }
          }
        }
        // 有效观测不足
        if (is_bad_mappts(i)) {
          for (auto e: edgeMappts[i]) this->removeEdge(e);
          edgeMappts[i].clear();
        }
      }
      return m;
    }

    void print_edge() {
      LOG(INFO) << "----- Edges -----";
      for (auto *e_: this->edges()) {
        auto e = static_cast<Edge *>(e_);
        if (e->level() == 1) continue;
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
        if (v) (*(iFramesBeg + i))->mPose.set_pose(T_imu_cam0 * g2o::toSE3(v->estimate()));
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

protected:

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
      // 获取 Frame 顶点
      auto vFrame = this->vertex(pFrame->mIdVex);
      if (!vFrame) return nullptr;
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
      eObs->setUserData(new g2o::ProjectEdgeData);
      // 设置顶点
      eObs->setId(pFrame->mIdVex * n_mappts + mp_id);
      eObs->setVertex(0, vFrame);
      if (!only_pose) eObs->setVertex(1, static_cast<g2o::VertexPointXYZ *>(vMappt));
      return eObs;
    }
};

}

#endif
