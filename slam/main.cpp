#include <thread>
#include <Eigen/Geometry>

#include "rcv.h"
#include "utils.h"

#include <condition_variable>

int main(int argc, char **argv) {

  std::condition_variable cond;

  Eigen::Isometry3d Tcw = Eigen::Isometry3d::Identity();

  std::cout << Tcw.matrix() << std::endl;

  Logger logger(argv);

  return 0;
}
