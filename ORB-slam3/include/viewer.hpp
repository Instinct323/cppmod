#ifndef ORBSLAM__VIEWER_HPP
#define ORBSLAM__VIEWER_HPP

#include "utils/glog.hpp"
#include "zjcv/slam/viewer.hpp"

namespace slam {


class Viewer {

public:
    ZJCV_SLAM_VIEWER_MEMBER

    ZJCV_SLAM_VIEWER_CONSTRUCTOR

    void run() {
      glog::Timer timer;
      auto &pSystem = this->mpSystem;
      auto &pTracker = pSystem->mpTracker;
      while (pSystem->mbRunning) {
        timer.reset();
        auto pFrame = pTracker->mpLastFrame;
        if (pFrame) {
          cv::Mat img0 = pFrame->mImg0.clone(), img1 = pFrame->mImg1.clone(), imgs;
          // Image 0
          pTracker->mpCam0->undistort(img0, img0);
          cv::cvtColor(img0, img0, cv::COLOR_GRAY2BGR);
          // cv::drawKeypoints(img0, pFrame->mvKps0, img0);
          // Image 1
          if (!img1.empty()) {
            pTracker->mpCam1->undistort(img1, img1);
            cv::cvtColor(img1, img1, cv::COLOR_GRAY2BGR);
            // cv::drawKeypoints(img1, pFrame->mvKps1, img1);
            // cv::hconcat(img0, img1, img0);
          }
          cv::drawMatches(img0, pFrame->mvKps0, img1, pFrame->mvKps1, pFrame->mStereoMatches, imgs);

          cv::imshow("Image", imgs);
        }
        int cost = static_cast<int>(timer.count() * 1e3);
        cv::waitKey(MAX(1, this->mDelay - cost));
      }
    }
};

}

#endif
