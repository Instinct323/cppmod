#ifndef ZJCV__SLAM__MAP_HPP
#define ZJCV__SLAM__MAP_HPP

#include <yaml-cpp/yaml.h>

#include "mappoint.hpp"
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

    ZJCV_BUILTIN parallel::atomic_ptr<std::vector<std::shared_ptr<Frame>>> mapKeyFrames;
    ZJCV_BUILTIN std::vector<std::weak_ptr<Mappoint>> mvpMappts, mvpTmpMappts;

    ZJCV_BUILTIN Map(System *pSystem) : mpSystem(pSystem), mapKeyFrames(new std::vector<std::shared_ptr<Frame>>) {}

    ZJCV_BUILTIN Mappoint::Ptr create_mappoint();

    ZJCV_BUILTIN void insert_keyframe(const std::shared_ptr<Frame> &pKF);

    ZJCV_BUILTIN void draw() const;

#ifdef ZJCV_ORB_SLAM
#endif

};

}

}

#endif
