#ifndef ZJCV__SLAM__MAPPOINT_HPP
#define ZJCV__SLAM__MAPPOINT_HPP

#include <map>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

namespace slam {

class Frame;

class Mappoint;

typedef std::pair<Frame *, int> Observation;

}


#define ZJCV_SLAM_MAPPOINT_MEMBER \
    typedef std::shared_ptr<slam::Mappoint> Ptr; \
    Eigen::Vector3f mPos; \
    std::vector<slam::Observation> mObs;


#define ZJCV_SLAM_MAPPOINT_FUNCDECL \
    void add_obs(slam::Frame *pFrame, const int &idx); \
    void erase_obs(slam::Frame *pFrame); \
    void process();


#define ZJCV_SLAM_MAPPOINT_IMPL \
    void slam::Mappoint::add_obs(slam::Frame *pFrame, const int &idx) { mObs.emplace_back(pFrame, idx); } \
    \
    void slam::Mappoint::erase_obs(slam::Frame *pFrame) { \
      mObs.erase(std::remove_if( \
          mObs.begin(), mObs.end(), \
          [pFrame](const slam::Observation &obs) { return obs.first == pFrame; }), mObs.end()); \
    }

#endif
