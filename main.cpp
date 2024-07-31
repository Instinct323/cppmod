#include <cstdlib>
#include <g2o/types/sba/edge_project_stereo_xyz_onlypose.h>

#include "utils/glog.hpp"

int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");
  glog::Logger logger(argv);

  return 0;
}
