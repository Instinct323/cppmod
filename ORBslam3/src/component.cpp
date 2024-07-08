#include "component/frame.hpp"
#include "component/system.hpp"
#include "component/tracker.hpp"
#include "component/viewer.hpp"

namespace slam {


Frame::Frame(Tracker *pTracker,
             const double &timestamp, const cv::Mat &img0, const cv::Mat &img1
) : mpTracker(pTracker), mTimestamp(timestamp), mImg0(img0), mImg1(img1) {
  // 设备与数据校验
  ASSERT((!mpTracker->mpCam1) == img1.empty(), "miss match between Camera1 and Image1")
  // 特征点提取、去畸
  if (!mpTracker->mpExtractor1) {
    mpTracker->mpCam0->monoORBfeatures(mpTracker->mpExtractor0.get(), mImg0, mvKps0, mDesc0);
  } else {
    mpTracker->mpCam0->stereoORBfeatures(mpTracker->mpCam1.get(), mpTracker->mpExtractor0.get(), mpTracker->mpExtractor1.get(),
                                         mImg0, mImg1, mvKps0, mvKps1, mDesc0, mDesc1, mStereoMatches);
  }
}


Tracker::Tracker(System *pSystem, YAML::Node cfg) :
    mpSystem(pSystem),
    mpCam0(camera::from_yaml(cfg["cam0"])), mpCam1(camera::from_yaml(cfg["cam1"])),
    mpExtractor0(ORB::Extractor::from_yaml(cfg["orb0"])), mpExtractor1(ORB::Extractor::from_yaml(cfg["orb1"])) {
  // 至少需要一个相机, 相机与特征提取器校验
  ASSERT(mpCam0 && mpExtractor0, "Camera0 not found")
  ASSERT((!mpCam1) == (!mpExtractor1), "miss match between Camera1 and Extractor1")
  // 相机类型校验
  if (mpCam1) {
    ASSERT(mpCam0->get_type() == mpCam1->get_type(), "Camera0 and Camera1 must be the same type")
    // Pinhole: 立体矫正
    /* if (mpCam0->get_type() == camera::CameraType::PINHOLE) {
      static_cast<camera::Pinhole *>(mpCam0.get())->stereo_rectify(static_cast<camera::Pinhole *>(mpCam1.get()));
    } */
  }
}


void Tracker::grab_image(const double &timestamp, const cv::Mat &img0, const cv::Mat &img1) {
  mpLastFrame = mpCurFrame;
  mpCurFrame = Frame::Ptr(new Frame(this, timestamp, img0, img1));
}


void Viewer::run() {
  glog::Timer timer;
  while (mpSystem->mbRunning) {
    timer.reset();
    Frame::Ptr pCurFrame = mpSystem->mpTracker->mpCurFrame;
    if (pCurFrame) {
      cv::Mat img0 = pCurFrame->mImg0.clone(), img1 = pCurFrame->mImg1.clone(), imgs;
      // Image 0
      pCurFrame->mpTracker->mpCam0->undistort(img0, img0);
      cv::cvtColor(img0, img0, cv::COLOR_GRAY2BGR);
      // cv::drawKeypoints(img0, pCurFrame->mvKps0, img0);
      // Image 1
      if (!img1.empty()) {
        pCurFrame->mpTracker->mpCam1->undistort(img1, img1);
        cv::cvtColor(img1, img1, cv::COLOR_GRAY2BGR);
        // cv::drawKeypoints(img1, pCurFrame->mvKps1, img1);
        // cv::hconcat(img0, img1, img0);
      }
      cv::drawMatches(img0, pCurFrame->mvKps0, img1, pCurFrame->mvKps1, pCurFrame->mStereoMatches, imgs);

      cv::imshow("Image", imgs);
    }
    int cost = static_cast<int>(timer.count() * 1e3);
    cv::waitKey(MAX(1, mDelay - cost));
  }
}

}
