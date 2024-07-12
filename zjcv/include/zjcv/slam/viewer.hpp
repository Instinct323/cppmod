#ifndef ZJCV__SLAM__VIEWER_HPP
#define ZJCV__SLAM__VIEWER_HPP

#include "utils/glog.hpp"

namespace slam {


template<typename System>
class ViewerBase {

protected:
    int mDelay;

public:
    System *mpSystem;

    explicit ViewerBase(System *pSystem, const YAML::Node &cfg) : mpSystem(pSystem) {
      int fps = cfg["viewer"]["fps"].as<int>();
      ASSERT(fps > 0, "Viewer: The delay must be greater than 0")
      mDelay = 1000 / fps;
    }

    ViewerBase(const ViewerBase &) = delete;

    virtual void run() = 0;
};

}

#endif
