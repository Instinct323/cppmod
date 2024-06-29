#include <cstdlib>

#include "utils/logging.hpp"
#include "utils/parallel.hpp"
#include "utils/std.hpp"


void mysleep(int y) {
  std::sleep(y);
  LOG(INFO) << "sleep for " << y;
}


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");
  Logger logger(argv);

  for (unsigned int i = 1; i < 20; i++) {
    auto t = parallel::thread_pool.emplace(2, mysleep, i);
  }
  parallel::thread_pool.join();

  return 0;
}
