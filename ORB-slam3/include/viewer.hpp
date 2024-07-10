#ifndef ORBSLAM__VIEWER_HPP
#define ORBSLAM__VIEWER_HPP

#include "utils/glog.hpp"

namespace slam {

class System;


class Viewer {
    const int mDelay;

public:
    typedef std::shared_ptr<Viewer> Ptr;

    const System *mpSystem;

    Viewer(System *pSystem, int fps = 45
    ) : mpSystem(pSystem), mDelay(1000 / fps) { ASSERT(fps > 0, "Viewer: The delay must be greater than 0")}

    Viewer(const Viewer &) = delete;

    void run();
};

}

#endif
