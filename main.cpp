#include <cstdlib>

#include "utils/glog.hpp"
#include "utils/parallel.hpp"


void mysleep(int y) {
  parallel::sleep(y);
  LOG(INFO) << "sleep for " << y;
}


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");
  glog::Logger logger(argv);

  for (unsigned int i = 1; i < 20; i++) {
    auto t = parallel::thread_pool.emplace(2, mysleep, i);
  }
  parallel::thread_pool.join();

  return 0;
}
