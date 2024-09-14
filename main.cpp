#include <cstdlib>
#include <librealsense2/rs.hpp>

#include "utils/glog.hpp"


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");
  glog::Logger logger(argv);

  rs2::context ctx;
  LOG(INFO) << "You have " << ctx.query_devices().size() << " RealSense devices connected" << std::endl;

  return 0;
}
