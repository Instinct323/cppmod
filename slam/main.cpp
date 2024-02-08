#include <thread>
#include <Eigen/Geometry>

#include "rcv.h"
#include "utils.h"


int main(int argc, char **argv) {
  Logger logger(argv);

  Eigen::MatrixXd A(6, 4);
  A.setRandom();

  Timer timer;
  auto svd = A.bdcSvd(Eigen::ComputeThinV);
  cout << svd.matrixV() << endl;

  LOG(INFO) << timer;

  return 0;
}
