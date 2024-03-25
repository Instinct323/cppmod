#include "utils.hpp"

#define ROOT "../data"


int main(int argc, char **argv) {
  Logger logger(argv);
  LOG(INFO) << __cplusplus;

  return 0;
}
