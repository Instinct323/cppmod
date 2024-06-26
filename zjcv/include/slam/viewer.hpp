#ifndef ZJCV__SLAM__VIEWER_HPP
#define ZJCV__SLAM__VIEWER_HPP

#include "utils/logging.hpp"

namespace slam {

class System;


class Viewer {

public:
    typedef std::shared_ptr<Viewer> Ptr;

    System *mpSystem;
    int mDelay;

    Viewer(System *pSystem, int delay = 30) : mpSystem(pSystem), mDelay(delay) {
      ASSERT(delay > 0, "Viewer: The delay must be greater than 0")
    }

    void run();
};

}

#endif
