#ifndef ZJCV__SLAM__VIEWER_HPP
#define ZJCV__SLAM__VIEWER_HPP

#include <yaml-cpp/yaml.h>

#include "utils/glog.hpp"

namespace slam { class Viewer; }


#define ZJCV_SLAM_VIEWER_MEMBER \
    typedef std::shared_ptr<slam::Viewer> Ptr; \
    slam::System *mpSystem; \
    int mDelay;


#define ZJCV_SLAM_VIEWER_FUNCDECL \
    explicit Viewer(slam::System *pSystem, const YAML::Node &cfg); \
    void run();


#define ZJCV_SLAM_VIEWER_IMPL \
    slam::Viewer::Viewer(slam::System *pSystem, const YAML::Node &cfg) : mpSystem(pSystem) { \
      int fps = cfg["viewer"]["fps"].as<int>(); \
      ASSERT(fps > 0, "Viewer: The delay must be greater than 0") \
      mDelay = 1000 / fps; \
    }

#endif
