#include "zjcv/slam.hpp"

namespace slam {

namespace feature {

void Map::draw() {
  Viewer &viewer = *mpSystem->mpViewer;
  int n = mvpKeyFrames.size();
  int &stride = viewer.sample_stride;
  for (int i = MAX(0, n - viewer.trail_size * stride); i < n; i += stride) {
    Frame::Ptr pKf = mvpKeyFrames[i];
    if (pKf) {
      pangolin::OpenGlMatrix T_wi(pKf->mPose.T_world_imu.matrix());
      pangolin::draw_imu(T_wi, viewer.imu_size);
    }
  }
}

}

}
