#include "utils.h"
#include <thread>

int main(int argc, char **argv) {
  Logger logger(argv);

  Timer timer;
  this_thread::sleep_for(chrono::duration<double>(1));
  LOG(INFO) << timer;
}
