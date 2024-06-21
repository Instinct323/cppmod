#ifndef ZJCV__EXTENSION__G2O_HPP
#define ZJCV__EXTENSION__G2O_HPP

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

namespace g2o {

// g2o 空序列化函数
#define G2O_EMPTY_SERIALIZE \
  virtual bool read(std::istream &in) override { return true; } \
  virtual bool write(std::ostream &out) const override { return true; }


/** @brief g2o 优化器 */
template<int p, int l,
    template<typename> class LinearSolverTp = LinearSolverDense,
    typename AlgorithmT = OptimizationAlgorithmLevenberg>

class Optimizer : public SparseOptimizer {
public:
    typedef BlockSolverPL<p, l> BlockSolverType;
    typedef LinearSolverTp<typename BlockSolverType::PoseMatrixType> LinearSolverType;

    Optimizer() {
      setAlgorithm(new AlgorithmT(
          make_unique<BlockSolverType>(
              make_unique<LinearSolverType>())));
    }
};

}

#endif
