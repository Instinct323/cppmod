#include <boost/format.hpp>

#include "frame.hpp"
#include "tracker.hpp"
#include "utils/glog.hpp"
#include "viewer.hpp"

namespace slam {

void Viewer::run() {
  glog::Timer timer;
  Tracker::Ptr pTracker = mpSystem->mpTracker;
  while (mpSystem->mbRunning) {
    timer.reset();
    Frame::Ptr pFrame = pTracker->mpLastFrame;
    if (pFrame) {
      cv::Mat img0 = pFrame->mImg0.clone(), img1 = pFrame->mImg1.clone(), img2;
      // Image 0
      pTracker->mpCam0->undistort(img0, img0);
      cv::cvtColor(img0, img0, cv::COLOR_GRAY2BGR);
      if (img1.empty()) {
        cv::drawKeypoints(img0, pFrame->mvKps0, img2);
      } else {
        // Image 1
        pTracker->mpCam1->undistort(img1, img1);
        cv::cvtColor(img1, img1, cv::COLOR_GRAY2BGR);
        cv::drawMatches(img0, pFrame->mvKps0, img1, pFrame->mvKps1, pFrame->mStereoMatches, img2);
      }
      cv::imshow("Image", img2);
    }
    int cost = static_cast<int>(timer.count() * 1e3);
    mpSystem->set_desc("view-FPS", std::to_string(1000 / MAX(mDelay, cost)));
    cv::waitKey(MAX(1, mDelay - cost));
  }
}

}
