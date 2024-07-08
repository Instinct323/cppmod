#ifndef ORBSLAM__SLAM__VIEWER_HPP
#define ORBSLAM__SLAM__VIEWER_HPP

#include "utils/glog.hpp"

namespace slam {

class System;


class Viewer {

public:
    typedef std::shared_ptr<Viewer> Ptr;

    System *mpSystem;
    int mDelay;

    Viewer(System *pSystem, int fps = 45
    ) : mpSystem(pSystem), mDelay(1000 / fps) { ASSERT(fps > 0, "Viewer: The delay must be greater than 0")}

    Viewer(const Viewer &) = delete;

    void run();
};

}

#endif
