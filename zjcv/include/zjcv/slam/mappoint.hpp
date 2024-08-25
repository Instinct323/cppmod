#ifndef ZJCV__SLAM__MAPPOINT_HPP
#define ZJCV__SLAM__MAPPOINT_HPP

#include <map>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <set>
#include <sophus/se3.hpp>

#include "frame.hpp"
#include "zjcv/zjcv.hpp"

namespace slam {

class System;

// 基于特征点
namespace feature {

typedef std::pair<std::weak_ptr<Frame>, int> Observation;

class Map;


class Mappoint {

public:
    ZJCV_BUILTIN typedef std::shared_ptr<Mappoint> Ptr;

    ZJCV_BUILTIN System *mpSystem;

    // Raw data
    ZJCV_BUILTIN size_t mIdVex = SIZE_MAX;
    ZJCV_BUILTIN parallel::atomic_ptr<std::vector<Observation>> apObs;

    // Processed data
    ZJCV_BUILTIN bool mbBad = true;
    ZJCV_BUILTIN Eigen::Vector3f mPos;

    ZJCV_BUILTIN void prune();

    ZJCV_BUILTIN void add_obs(const Frame::Ptr &pFrame, const int &idx);

    ZJCV_BUILTIN void erase_obs(const Frame::Ptr &pFrame);

    ZJCV_BUILTIN void set_pos(const Eigen::Vector3f &pos);

    ZJCV_BUILTIN void set_invalid(bool bad = true) { mbBad = bad; }

    ZJCV_BUILTIN bool is_invalid() const { return mbBad; }

    ZJCV_BUILTIN void clear();

    ZJCV_BUILTIN void merge(Ptr &shared_this, Ptr &other);

    ZJCV_BUILTIN friend class Map;

protected:
    // Created by Frame, cleaned up by Map
    ZJCV_BUILTIN explicit Mappoint(System *pSystem
    ) : mpSystem(pSystem), apObs(new std::vector<Observation>) {}

#ifdef ZJCV_ORB_SLAM
#endif

};

}

}

#endif
