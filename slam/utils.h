#pragma once

#include <chrono>
#include <glog/logging.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <iostream>


/** @brief 日志 */
class Logger {
public:
    explicit Logger(char **argv) {
      google::InitGoogleLogging(argv[0]);
      FLAGS_logtostderr = true;
      FLAGS_minloglevel = google::INFO;
    }

    ~Logger() { google::ShutdownGoogleLogging(); }
};


/** @brief 计时器 */
class Timer {

public:
    typedef std::chrono::steady_clock Clock;
    typedef Clock::time_point Timepoint;
    typedef std::chrono::duration<double> Duration;

    Timepoint t0;

    Timer() { t0 = Clock::now(); }

protected:
    friend std::ostream &operator<<(std::ostream &os, const Timer &timer) {
      Timepoint t1 = Clock::now();
      Duration time_used = std::chrono::duration_cast<Duration>(t1 - timer.t0);
      return (os << time_used.count());
    }
};

#include <g2o/types/slam3d/types_slam3d.h>

typedef g2o::VertexSE3 VertexPose;


///// 仅估计位姿的一元边
//class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2, Vec2, VertexPose> {
//public:
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
//
//    EdgeProjectionPoseOnly(const Vec3 &pos, const Mat33 &K)
//        : _pos3d(pos), _K(K) {}
//
//    virtual void computeError() override {
//      const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
//      SE3 T = v->estimate();
//      Vec3 pos_pixel = _K * (T * _pos3d);
//      pos_pixel /= pos_pixel[2];
//      _error = _measurement - pos_pixel.head<2>();
//    }
//
//    virtual bool read(std::istream &in) override { return true; }
//
//    virtual bool write(std::ostream &out) const override { return true; }
//
//private:
//    Vec3 _pos3d;
//    Mat33 _K;
//};
