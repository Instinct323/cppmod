#include "slam/frame.hpp"
#include "slam/system.hpp"
#include "slam/tracker.hpp"
#include "slam/viewer.hpp"

#include "boost/thread.hpp"
#include <thread>

namespace slam {


Frame::Frame(Tracker *pTracker,
             const double &timestamp, const cv::Mat &img0, const cv::Mat &img1,
             const std::vector<IMU::Sample> &vImu
) : mTimestamp(timestamp), mImg0(img0), mImg1(img1), mvImu(vImu) {
  // 设备与数据校验
  ASSERT((!pTracker->mpCam1) == img1.empty(), "miss match between Camera1 and Image1")
  ASSERT((!pTracker->mpIMU) == vImu.empty(), "miss match between IMU device and IMU samples")
  // 特征点提取、去畸
  boost::thread t0(std::bind(&ORB::Extractor::detect_and_compute,
                             pTracker->mpExtractor0.get(), mImg0, cv::Mat(), std::ref(mvKps0), std::ref(mDesc0)));
  if (pTracker->mpExtractor1) {
    pTracker->mpExtractor1->detect_and_compute(mImg1, cv::Mat(), mvKps1, mDesc1);
    pTracker->mpCam1->undistort(mvKps1, mvKps1);
  }
  t0.join();
  pTracker->mpCam0->undistort(mvKps0, mvKps0);
}


Tracker::Tracker(System *pSystem, YAML::Node cfg) :
    mpSystem(pSystem),
    mpCam0(camera::from_yaml(cfg["cam0"])), mpCam1(camera::from_yaml(cfg["cam1"])),
    mpIMU(IMU::Device::from_yaml(cfg["imu"])),
    mpExtractor0(ORB::Extractor::from_yaml(cfg["orb0"])), mpExtractor1(ORB::Extractor::from_yaml(cfg["orb1"])) {
  // 至少需要一个相机, 相机与特征提取器校验
  ASSERT(mpCam0 && mpExtractor0, "Camera0 not found")
  ASSERT((!mpCam1) == (!mpExtractor1), "miss match between Camera1 and Extractor1")
  // 相机类型校验
  if (mpCam1) {
    ASSERT(mpCam0->get_type() == mpCam1->get_type(), "Camera0 and Camera1 must be the same type")
    // Pinhole: 立体矫正
    if (mpCam0->get_type() == camera::CameraType::PINHOLE) {
      static_cast<camera::Pinhole *>(mpCam0.get())->stereo_rectify(static_cast<camera::Pinhole *>(mpCam1.get()));
    }
  }
};


void Tracker::grad_image_and_imu(const double &timestamp, const cv::Mat &img0, const cv::Mat &img1,
                                 const std::vector<IMU::Sample> &vImu) {
  mpLastFrame = mpCurFrame;
  mpCurFrame = Frame::Ptr(new Frame(this, timestamp, img0, img1, vImu));
}


void Viewer::run() {
  while (mpSystem->mbRunning) {
    Frame::Ptr pCurFrame = mpSystem->mpTracker->mpCurFrame;
    if (pCurFrame) {
      cv::Mat img0 = pCurFrame->mImg0.clone(), img1 = pCurFrame->mImg1.clone();
      // Image 0
      cv::cvtColor(img0, img0, cv::COLOR_GRAY2BGR);
      cv::drawKeypoints(img0, pCurFrame->mvKps0, img0);
      cv::imshow("Image 0", img0);
      // Image 1
      if (!img1.empty()) {
        cv::cvtColor(img1, img1, cv::COLOR_GRAY2BGR);
        cv::drawKeypoints(img1, pCurFrame->mvKps1, img1);
        cv::imshow("Image 1", img1);
      }
      cv::waitKey(mDelay);
    }
  }
}

}
