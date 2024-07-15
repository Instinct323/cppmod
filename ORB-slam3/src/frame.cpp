#include "frame.hpp"
#include "tracker.hpp"


namespace slam {


void Frame::process() {
  Tracker::Ptr pTracker = mpSystem->mpTracker;
  Ptr pLastFrame = pTracker->mpLastFrame;
  camera::Base::Ptr pCam0 = pTracker->mpCam0, pCam1 = pTracker->mpCam1;
  // IMU 预测位姿
  if (pTracker->is_inertial()) {
    if (pLastFrame) {
      mPose.predict_from(pTracker->mpLastFrame->mPose, pTracker->mpIMUpreint.get());
    } else {
      mPose.set_zero();
    }
  }
  // 特征点提取、去畸
  if (pTracker->is_monocular()) {
    pCam0->monoORBfeatures(pTracker->mpExtractor0.get(), mImg0, mvKps0, mDesc0);
  } else {
    pCam0->stereoORBfeatures(pTracker->mpCam1.get(), pTracker->mpExtractor0.get(), pTracker->mpExtractor1.get(),
                             mImg0, mImg1, mvKps0, mvKps1, mDesc0, mDesc1, mStereoMatches);
    bool is_fisheye = pCam0->get_type() == camera::KANNALA_BRANDT;
    for (auto &m: mStereoMatches) {
      Eigen::Vector3f P0_cam0 = pCam0->unproject(mvKps0[m.queryIdx].pt),
          P1_cam1 = pCam1->unproject(mvKps1[m.trainIdx].pt);
      // 鱼眼相机余弦判断
      if (is_fisheye) {
        Eigen::Vector3f P1_cam0 = pTracker->T_cam0_cam1 * P1_cam1;
        if (Eigen::cos(P0_cam0, P1_cam0) < 0.9998) {
          // todo: 保留并三角剖分
        }
      }
    }
  }
}


}
