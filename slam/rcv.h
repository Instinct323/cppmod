#pragma once

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#define G2O_EMPTY_SERIALIZE \
  virtual bool read(std::istream &in) override { return true; } \
  virtual bool write(std::ostream &out) const override { return true; }

typedef Sophus::SE3d SE3;
typedef Eigen::Matrix3d Mat33;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector2d Vec2;


/** @brief 相机 */
class Camera {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Camera> Ptr;

    double fx = 1, fy = 1, cx = 0, cy = 0;
    SE3 Tcr, Trc, Trw, Twr, Tcw, Twc;   // camera, robot, world

    // 返回内参
    Mat33 K() const {
      Mat33 k;
      k << fx, 0, cx, 0, fy, cy, 0, 0, 1;
      return k;
    }

    // 变换矩阵
    void set_Tcr(const SE3 &T) {
      Tcr = T;
      Trc = T.inverse();
      update_Tcw();
    }

    void set_Trw(const SE3 &T) {
      Trw = T;
      Twr = T.inverse();
      update_Tcw();
    }

    void update_Tcw() {
      Tcw = Tcr * Trw;
      Twc = Tcw.inverse();
    }

    // 坐标变换
    Vec2 camera2pixel(const Vec3 &p_c) const {
      return {fx * p_c(0) / p_c(2) + cx,
              fy * p_c(1) / p_c(2) + cy};
    }

    Vec3 pixel2camera(const Vec2 &p_p, double depth) const {
      return {(p_p(0) - cx) / fx * depth,
              (p_p(1) - cy) / fy * depth,
              depth};
    }

    Vec3 world2camera(const Vec3 &p_w) const { return Tcw * p_w; }

    Vec3 camera2world(const Vec3 &p_c) const { return Twc * p_c; }

    Vec2 world2pixel(const Vec3 &p_w) const { return camera2pixel(world2camera(p_w)); }

    Vec3 pixel2world(const Vec2 &p_p, double depth) const { return camera2world(pixel2camera(p_p, depth)); }
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
