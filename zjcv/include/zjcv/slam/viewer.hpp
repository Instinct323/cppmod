#ifndef ZJCV__SLAM__VIEWER_HPP
#define ZJCV__SLAM__VIEWER_HPP

#include "utils/glog.hpp"

namespace slam {

class Viewer;

}

#define ZJCV_SLAM_VIEWER_MEMBER \
    typedef std::shared_ptr<Viewer> Ptr; \
    System *mpSystem; \
    int mDelay;


#define ZJCV_SLAM_VIEWER_CONSTRUCTOR \
    explicit Viewer(System *pSystem, const YAML::Node &cfg) : mpSystem(pSystem) { \
      int fps = cfg["viewer"]["fps"].as<int>(); \
      ASSERT(fps > 0, "Viewer: The delay must be greater than 0") \
      mDelay = 1000 / fps; \
    }

#endif
