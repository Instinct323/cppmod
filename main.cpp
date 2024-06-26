#include <cstdlib>

#include "utils/logging.hpp"


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");
  Logger logger(argv);

  return 0;
}
