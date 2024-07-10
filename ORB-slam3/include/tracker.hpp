#ifndef ORBSLAM__TRACKER_HPP
#define ORBSLAM__TRACKER_HPP

#include <yaml-cpp/yaml.h>

#include "utils/orb.hpp"
#include "zjcv/camera.hpp"
#include "zjcv/slam/tracker.hpp"

namespace slam {


template<typename System>
class Tracker : public TrackerBase<System> {

public:
    // Extractors
    const ORB::Extractor::Ptr mpExtractor0, mpExtractor1;

    explicit Tracker(System *pSystem, const YAML::Node &cfg) :
        TrackerBase<System>(pSystem, cfg),
        mpExtractor0(ORB::Extractor::from_yaml(cfg["orb0"])),
        mpExtractor1(ORB::Extractor::from_yaml(cfg["orb1"])) {
      ASSERT(mpExtractor0, "Extractor0 not found")
      ASSERT((!this->mpCam1) == (!mpExtractor1), "miss match between Camera1 and Extractor1")
    }

    virtual void run() override;
};

}

#endif
