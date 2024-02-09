#pragma once

#include <sophus/se3.hpp>

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include "camera.h"
#include "utils.h"

#define G2O_EMPTY_SERIALIZE \
  virtual bool read(std::istream &in) override { return true; } \
  virtual bool write(std::ostream &out) const override { return true; }


/** @brief g2o 优化器 */
template<int p, int l,
    template<typename> class LinearSolverTp = g2o::LinearSolverDense,
    typename AlgorithmT = g2o::OptimizationAlgorithmLevenberg>

class Optimizer : public g2o::SparseOptimizer {
public:
    typedef g2o::BlockSolverPL<p, l> BlockSolverType;
    typedef LinearSolverTp<typename BlockSolverType::PoseMatrixType> LinearSolverType;

    Optimizer() {
      setAlgorithm(new AlgorithmT(
          g2o::make_unique<BlockSolverType>(
              g2o::make_unique<LinearSolverType>())));
    }
};


/** @brief 位姿顶点 */
class VertexPose : public g2o::BaseVertex<6, SE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    G2O_EMPTY_SERIALIZE;

    void setToOriginImpl() override { _estimate = SE3(); }

    void oplusImpl(const double *update) override {
      Eigen::Matrix<double, 6, 1> update_eigen(update);
      _estimate = SE3::exp(update_eigen) * _estimate;
    }
};


/**
 * @brief 估计位姿的一元边
 * @param p_w 世界坐标系坐标
 * @var _measurement 像素坐标系坐标
 */
class EdgePose : public g2o::BaseUnaryEdge<2, Vec2, VertexPose> {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    G2O_EMPTY_SERIALIZE;

    Vec3 *p_w;
    Camera::Ptr camera;

    EdgePose(Vec3 *p_w, Camera::Ptr camera) : p_w(p_w), camera(camera) {}

    void computeError() override {
      const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
      SE3 Tcw = v->estimate();
      Vec2 p_p = camera->camera2pixel(Tcw * *p_w);
      _error = _measurement - p_p;
    }
};
