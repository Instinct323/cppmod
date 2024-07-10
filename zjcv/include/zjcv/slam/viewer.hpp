#ifndef ORBSLAM__VIEWER_HPP
#define ORBSLAM__VIEWER_HPP

#include "utils/glog.hpp"

namespace slam {


template<typename System>
class ViewerBase {
    const int mDelay;

public:
    const System *mpSystem;

    ViewerBase(System *pSystem, int fps = 45
    ) : mpSystem(pSystem), mDelay(1000 / fps) { ASSERT(fps > 0, "Viewer: The delay must be greater than 0")}

    ViewerBase(const ViewerBase &) = delete;

    virtual void run() = 0;
};

}

#endif
