#include "utils.h"
#include "loop.h"

int main(int argc, char **argv) {
  Logger logger(argv);

  LOG(INFO) << INIT_SUCCESS;

  return 0;
}
