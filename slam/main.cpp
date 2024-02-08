#include <thread>
#include <Eigen/Geometry>

#include "rcv.h"
#include "utils.h"

#include <condition_variable>

int main(int argc, char **argv) {

  std::condition_variable cond;

  Logger logger(argv);

  LOG(INFO) << x.matrix();

  return 0;
}
