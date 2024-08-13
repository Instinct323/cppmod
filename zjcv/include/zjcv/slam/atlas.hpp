#ifndef ZJCV__SLAM__ATLAS_HPP
#define ZJCV__SLAM__ATLAS_HPP

#include <yaml-cpp/yaml.h>

#include "map.hpp"
#include "zjcv/zjcv.hpp"

namespace slam {

class System;


class Atlas {

public:
    ZJCV_BUILTIN typedef std::shared_ptr<Atlas> Ptr;

    ZJCV_BUILTIN System *mpSystem;

    // Config
    ZJCV_BUILTIN const int LOCAL_FRAMES;

    // Memory
    ZJCV_BUILTIN bool mbSleep = false;
    ZJCV_BUILTIN feature::Map::Ptr mpCurMap;
    ZJCV_BUILTIN std::vector<feature::Map::Ptr> mvpMaps;

    ZJCV_BUILTIN explicit Atlas(System *pSystem, const YAML::Node &cfg
    ) : mpSystem(pSystem), LOCAL_FRAMES(cfg["local_frames"].as<int>()) { create_map(); }

    ZJCV_BUILTIN feature::Map::Ptr create_map();

    ZJCV_BUILTIN void set_sleep() { mbSleep = true; }

    ZJCV_BUILTIN void run();

#ifdef ZJCV_ORB_SLAM
#endif

};

}

#endif
