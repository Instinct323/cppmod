#ifndef ZJCV__SLAM__ATLAS_HPP
#define ZJCV__SLAM__ATLAS_HPP

#include <yaml-cpp/yaml.h>

#include "map.hpp"
#include "zjcv/zjcv.hpp"

namespace slam {

using namespace feature;

class System;


class Atlas {

public:
    ZJCV_BUILTIN typedef std::shared_ptr<Atlas> Ptr;

    ZJCV_BUILTIN System *mpSystem;
    ZJCV_BUILTIN Map::Ptr mpCurMap;
    ZJCV_BUILTIN std::vector<Map::Ptr> mvpMaps;

    ZJCV_BUILTIN explicit Atlas(System *pSystem, const YAML::Node &cfg) : mpSystem(pSystem) { create_map(); }

    ZJCV_BUILTIN Map::Ptr create_map();

#ifdef ZJCV_ORB_SLAM
#endif

};

}

#endif
