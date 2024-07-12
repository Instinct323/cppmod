#ifndef ZJCV__SLAM__MAPPOINT_HPP
#define ZJCV__SLAM__MAPPOINT_HPP

#include <map>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

namespace slam {


template<typename System>
class Mappoint {

public:
    typedef std::pair<Sophus::SE3d *, cv::Point2f *> Observation;

    Eigen::Vector3d mPos;
    std::multimap<typename System::Frame *, Observation> mObservations;

    explicit Mappoint(const Eigen::Vector3d &pos) : mPos(pos) {}

    void add_obs(typename System::Frame *pFrame, Sophus::SE3d *pPose, cv::Point2f *pKp) {
      mObservations.insert({pFrame, std::make_pair(pPose, pKp)});
    }

    void erase_obs(typename System::Frame *pFrame) {
      while (true) {
        auto it = mObservations.find(pFrame);
        if (it == mObservations.end()) break;
        mObservations.erase(it);
      }
    }
};

}

#endif
