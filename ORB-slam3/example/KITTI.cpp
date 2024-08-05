#include <boost/format.hpp>
#include <filesystem>

#include "utils/cv.hpp"
#include "utils/fbow.hpp"
#include "utils/indicators.hpp"
#include "utils/glog.hpp"
#include "utils/math.hpp"
#include "zjcv/dataset/tum_vi.hpp"
#include "zjcv/imu.hpp"
#include "zjcv/slam.hpp"


// Tracking
void slam::Tracker::run(){

}


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");
  glog::Logger logger(argv);
  LOG(INFO) << "Thread pool size: " << parallel::thread_pool_size;

  // config
  YAML::Node cfg = YAML::LoadFile("/home/workbench/cppmod/ORB-slam3/example/KITTI.yaml");

  return 0;
  slam::System system(cfg);
  system.mpTracker->reload(cfg["tracker"]);
  system.run();
  system.mThreads["tracker"]->join();
  system.stop();
  return 0;
}
