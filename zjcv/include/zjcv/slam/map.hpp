#ifndef ZJCV__SLAM__MAP_HPP
#define ZJCV__SLAM__MAP_HPP

#include <yaml-cpp/yaml.h>

namespace slam {

class System;


class Map {

public:
    typedef std::shared_ptr<Map> Ptr;

    System *mpSystem;

    Map(System *pSystem, const YAML::Node &cfg) : mpSystem(pSystem) {}
};

}

#endif
