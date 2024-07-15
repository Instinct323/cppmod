#ifndef ZJCV__SLAM__MAPPOINT_HPP
#define ZJCV__SLAM__MAPPOINT_HPP

#include <map>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

namespace slam {

class MapPoint;

}


#define ZJCV_SLAM_MAPPOINT_MEMBER \
  typedef std::shared_ptr<MapPoint> Ptr; \
  Eigen::Vector3f mPos;

#endif
