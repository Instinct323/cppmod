#ifndef ZJCV__SLAM__MAP_HPP
#define ZJCV__SLAM__MAP_HPP

#include <yaml-cpp/yaml.h>

#include "zjcv/zjcv.hpp"

namespace slam {

class System;

// 基于特征点
namespace feature {

class Frame;


class Map {

public:
    ZJCV_BUILTIN typedef std::shared_ptr<Map> Ptr;

    ZJCV_BUILTIN System *mpSystem;
    ZJCV_BUILTIN std::vector<std::shared_ptr<Frame>> mvpKeyFrames;

    ZJCV_BUILTIN Map(System *pSystem) : mpSystem(pSystem) {}

    ZJCV_BUILTIN void insert_keyframe(const std::shared_ptr<Frame>& pKF) { mvpKeyFrames.push_back(pKF); }

    ZJCV_CUSTOM void draw() const;

#ifdef ZJCV_ORB_SLAM
#endif

};

}

}

#endif
