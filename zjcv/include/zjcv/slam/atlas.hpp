#ifndef ZJCV__SLAM__ATLAS_HPP
#define ZJCV__SLAM__ATLAS_HPP

#include <yaml-cpp/yaml.h>

#include "map.hpp"

namespace slam {

class System;


class Atlas {

public:
    typedef std::shared_ptr<Atlas> Ptr;

    System *mpSystem;

    Map::Ptr mpCurMap;
    std::vector<Map::Ptr> mvpMaps;

    Atlas(System *pSystem, const YAML::Node &cfg) : mpSystem(pSystem) { create_map(); }

    Map::Ptr create_map() {
      mpCurMap = std::make_shared<Map>(mpSystem);
      mvpMaps.push_back(mpCurMap);
      return mpCurMap;
    }
};

}

#endif
