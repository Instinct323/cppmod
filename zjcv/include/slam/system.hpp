#ifndef ZJCV__SLAM__SYSTEM_HPP
#define ZJCV__SLAM__SYSTEM_HPP

#include "tracker.hpp"

namespace slam {


class System {

public:
    Tracker::Ptr mpTracker;

    explicit System(YAML::Node cfg) : mpTracker(new Tracker(cfg)) {};
};

}

#endif
