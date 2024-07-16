#include <cstdlib>

#include "utils/glog.hpp"


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");
  glog::Logger logger(argv);

  return 0;
}
