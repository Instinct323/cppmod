#ifndef UTILS__G2O_HPP
#define UTILS__G2O_HPP

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/se3quat.h>
#include <sophus/se3.hpp>

// g2o 空序列化函数
#define G2O_EMPTY_SERIALIZE \
  virtual bool read(std::istream &in) override { return true; } \
  virtual bool write(std::ostream &out) const override { return true; }

namespace g2o {

// g2o -> Sophus
Sophus::SE3f toSE3(const g2o::SE3Quat &pose);


// g2o 优化器
template<int p, int l,
    template<typename> class LinearSolverTp,
    typename AlgorithmT = OptimizationAlgorithmLevenberg
>
class Optimizer : public SparseOptimizer {

public:
    typedef BlockSolverPL<p, l> BlockSolverType;
    typedef LinearSolverTp<typename BlockSolverType::PoseMatrixType> LinearSolverType;

    Optimizer() : SparseOptimizer() {
      auto lin_solver = make_unique<LinearSolverType>();
      std::unique_ptr<BlockSolverType> block_solver(new BlockSolverType(std::move(lin_solver)));
      setAlgorithm(new AlgorithmT(std::move(block_solver)));
    }
};

}

#endif
