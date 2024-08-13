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
    ZJCV_BUILTIN std::mutex mPosesMutex;

    // Memory
    ZJCV_BUILTIN int miKeyFrame = 0, miMappt = 0;
    ZJCV_BUILTIN parallel::atomic_ptr<std::vector<std::shared_ptr<Frame>>> apKeyFrames;
    ZJCV_BUILTIN parallel::atomic_ptr<std::vector<std::weak_ptr<Mappoint>>> apMappts, apTmpMappts;

    ZJCV_BUILTIN explicit Map(System *pSystem
    ) : mpSystem(pSystem), apKeyFrames(new std::vector<std::shared_ptr<Frame>>),
        apMappts(new std::vector<std::weak_ptr<Mappoint>>), apTmpMappts(new std::vector<std::weak_ptr<Mappoint>>) {}

    ZJCV_BUILTIN Mappoint::Ptr create_mappoint();

    ZJCV_BUILTIN bool insert_keyframe(const std::shared_ptr<Frame> &pKF);

    ZJCV_BUILTIN void prune_mappts(int begin);

    ZJCV_BUILTIN void local_mapping();

    ZJCV_BUILTIN void draw(Frame::Ptr pCurFrame = nullptr) const;

#ifdef ZJCV_ORB_SLAM
#endif

};

}

}

#endif
