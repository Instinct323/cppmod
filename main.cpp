#include "utils.h"

#define ROOT "../data"


int main(int argc, char **argv) {
  setenv("DISPLAY", "host.docker.internal:0", 1);
  Logger logger(argv);

  LOG(INFO) << __cplusplus;

  return 0;
}
