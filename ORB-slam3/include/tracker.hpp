#ifndef ORBSLAM__TRACKER_HPP
#define ORBSLAM__TRACKER_HPP

#include "frame.hpp"
#include "utils/orb.hpp"
#include "zjcv/slam/system.hpp"

namespace slam {

class System;


class Tracker {

public:
    ZJCV_SLAM_TRACKER_MEMBER

    ZJCV_SLAM_TRACKER_FUNCDECL

    // Extractors
    ORB::Extractor::Ptr mpExtractor0, mpExtractor1;

    void reload(const YAML::Node &cfg) {
      mpExtractor0 = ORB::Extractor::from_yaml(cfg["orb0"]);
      mpExtractor1 = ORB::Extractor::from_yaml(cfg["orb1"]);
      ASSERT(mpExtractor0, "Extractor0 not found")
      ASSERT((!mpCam1) == (!mpExtractor1), "miss match between Camera1 and Extractor1")
    }
};

}

#endif
