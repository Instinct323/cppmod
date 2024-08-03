#ifndef ZJCV__SLAM__OPTIMIZE_HPP
#define ZJCV__SLAM__OPTIMIZE_HPP

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>

#include "frame.hpp"
#include "utils/g2o.hpp"

#define G2O_WITH_PWORLD \
  Eigen::Vector3d Pw; \
  void set_pworld(const Eigen::Vector3d &pw) { Pw = pw; }

namespace g2o {


class EdgeSE3ProjectDNP : public BaseBinaryEdge<2, Eigen::Vector2d, VertexSE3Expmap, VertexPointXYZ> {

public:
    G2O_EMPTY_SERIALIZE

    virtual void computeError() override {
      const g2o::SE3Quat &T_cw = static_cast<VertexSE3Expmap *>(_vertices[0])->estimate();
      const Eigen::Vector3d &Pw = static_cast<VertexPointXYZ *>(_vertices[1])->estimate();
    }
};


class EdgeSE3ProjectDNPonlyPose : public BaseUnaryEdge<2, Eigen::Vector2d, VertexSE3Expmap> {

public:
    G2O_EMPTY_SERIALIZE

    G2O_WITH_PWORLD

    virtual void computeError() override {
      const g2o::SE3Quat &T_cw = static_cast<VertexSE3Expmap *>(_vertices[0])->estimate();
    }
};


class EdgeStereoSE3ProjectDNP : public BaseBinaryEdge<3, Eigen::Vector3d, VertexSE3Expmap, VertexPointXYZ> {

public:
    G2O_EMPTY_SERIALIZE

    virtual void computeError() override {
      const g2o::SE3Quat &T_cw = static_cast<VertexSE3Expmap *>(_vertices[0])->estimate();
      const Eigen::Vector3d &Pw = static_cast<VertexPointXYZ *>(_vertices[1])->estimate();
    }
};


class EdgeStereoSE3ProjectDNPonlyPose : public BaseUnaryEdge<3, Eigen::Vector3d, VertexSE3Expmap> {

public:
    G2O_EMPTY_SERIALIZE

    G2O_WITH_PWORLD

    virtual void computeError() override {
      const g2o::SE3Quat &T_cw = static_cast<VertexSE3Expmap *>(_vertices[0])->estimate();
    }
};


}


namespace slam::feature {

void optimize_pose(const std::shared_ptr<Frame> &pFrame);


// 光束法平差
template<template<typename> class LinearSolverTp>
class BundleAdjustment : public g2o::Optimizer<6, 3, LinearSolverTp> {

    std::vector<std::shared_ptr<Frame>> *pFrames;
    std::vector<std::weak_ptr<Mappoint>> *pMappts;
    bool only_pose;

    size_t n_frames = 0, n_mappts = 0, id_vex = 0;

public:
    using g2o::SparseOptimizer::Vertex;
    using g2o::SparseOptimizer::Edge;

    BundleAdjustment(std::vector<std::shared_ptr<Frame>> *pFrames,
                     std::vector<std::weak_ptr<Mappoint>> *pMappts,
                     bool only_pose
    ) : g2o::Optimizer<6, 3, LinearSolverTp>(),
        pFrames(pFrames), pMappts(pMappts), only_pose(only_pose),
        n_frames(pFrames->size()), n_mappts(pMappts->size()), id_vex(n_frames + n_mappts) {
      // Vertex: 帧位姿
      for (size_t i = 0; i < n_frames; ++i) {
        auto vFrame = add_frame(pFrames->at(i), i);
        vFrame->setFixed(false);
      }

      for (size_t i = 0; i < pMappts->size(); ++i) {
        if (pMappts->at(i).expired()) continue;
        auto pMappt = pMappts->at(i).lock();
        Eigen::Vector3d Pw = pMappt->mPos.cast<double>();
        // Vertex: 地图点
        Vertex *vMappt = add_mappt(pMappt, n_frames + i);
        // Edge: 地图点投影
        {
          parallel::ScopedLock lock(pMappt->mapObs.mutex);
          for (auto obs: *pMappt->mapObs) {
            Edge *eObs = add_observation(Pw, obs, vMappt);
            // todo: 想想怎么做离群点筛除
          }
        }
      }
    }

    Vertex *add_frame(Frame::Ptr &pFrame, size_t id) {
      pFrame->mIdVex = id;
      auto *vFrame = new g2o::VertexSE3Expmap;
      vFrame->setEstimate(Sophus::toG2O(pFrame->mPose.T_imu_world));
      vFrame->setId(id);
      this->addVertex(vFrame);
      return static_cast<Vertex *>(vFrame);
    }

    Vertex *add_mappt(Mappoint::Ptr &pMappt, size_t id) {
      if (only_pose) return nullptr;
      auto vMappt = new g2o::VertexPointXYZ;
      vMappt->setEstimate(pMappt->mPos.cast<double>());
      vMappt->setId(id);
      vMappt->setFixed(false);
      this->addVertex(vMappt);
      return static_cast<Vertex *>(vMappt);
    }

    Edge *add_observation(const Eigen::Vector3d &Pw, Observation &obs, Vertex *vMappt) {
      auto [wpf, idx] = obs;
      if (wpf.expired()) return nullptr;
      auto pFrame = wpf.lock();
      Edge *eObs;
      Eigen::Vector3d Pc = pFrame->mvUnprojs0[idx].cast<double>();
      // 双目标记, 双目观测的 z 为实际值的相反数
      if (Pc[2] < 0) {
        Pc[2] = -Pc[2];
        if (only_pose) {
          // todo: 信息矩阵, 鲁棒核
          auto eTmp = new g2o::EdgeStereoSE3ProjectDNPonlyPose;
          eTmp->set_pworld(Pw);
          eTmp->setMeasurement(Pc);
          eObs = static_cast<Edge *>(eTmp);
        } else {
          auto eTmp = new g2o::EdgeStereoSE3ProjectDNP;
          eTmp->setMeasurement(Pc);
          eObs = static_cast<Edge *>(eTmp);
        }
      } else {
        if (only_pose) {
          auto eTmp = new g2o::EdgeSE3ProjectDNPonlyPose;
          eTmp->set_pworld(Pw);
          eTmp->setMeasurement(Pc.head<2>());
          eObs = static_cast<Edge *>(eTmp);
        } else {
          auto eTmp = new g2o::EdgeSE3ProjectDNP;
          eTmp->setMeasurement(Pc.head<2>());
          eObs = static_cast<Edge *>(eTmp);
        }
      }
      // 设置顶点
      auto vFrame = static_cast<g2o::VertexSE3Expmap *>(this->vertex(pFrame->mIdVex));
      if (!vFrame) {
        vFrame = add_frame(pFrame, id_vex);
        vFrame->setFixed(true);
        id_vex++;
      }
      eObs->setVertex(0, vFrame);
      if (!only_pose) eObs->setVertex(1, static_cast<g2o::VertexPointXYZ *>(vMappt));
    }

    void apply_result() {
      // todo: 离群标记
      // Frame: 位姿
      for (size_t i = 0; i < n_frames; ++i) {
        auto v = static_cast<g2o::VertexSE3Expmap *>(this->vertex(i));
        if (v) pFrames->at(i)->mPose.set_pose(g2o::toSE3(v->estimate()));
      }
      // Mappoint: 位置
      for (size_t i = 0; i < n_mappts; ++i) {
        if (pMappts->at(i).expired()) continue;
        auto pMappt = pMappts->at(i).lock();
        auto v = static_cast<g2o::VertexPointXYZ *>(this->vertex(n_frames + i));
        if (v) pMappt->mPos = v->estimate().cast<float>();
      }
    }
};

}

#endif
