#ifndef ZJCV__SLAM__MAP_HPP
#define ZJCV__SLAM__MAP_HPP

#include <yaml-cpp/yaml.h>

namespace slam {

class Frame;

class System;


class Map {

public:
    typedef std::shared_ptr<slam::Map> Ptr;

    slam::System *mpSystem;
    std::vector<std::shared_ptr<slam::Frame>> mvpKeyFrames;

    Map(slam::System *pSystem) : mpSystem(pSystem) {}

    void insert_keyframe(std::shared_ptr<slam::Frame> pKF) { mvpKeyFrames.push_back(pKF); }
};

}

#endif
