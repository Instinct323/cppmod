#include "slam/system.hpp"
#include "slam/tracker.hpp"

namespace slam {


void Tracker::GrabImageAndImu(const double &timestamp, const cv::Mat &img0, const cv::Mat &img1,
                              const std::vector<IMU::Sample> &vImu) {
  ASSERT((mpCam1 == nullptr) == img1.empty(), "miss match between Camera1 and Image1")
  ASSERT((mpIMU == nullptr) == vImu.empty(), "miss match between IMU device and IMU samples")
  // todo
}

}
