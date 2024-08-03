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


class Mappoint {

public:
    ZJCV_BUILTIN typedef std::shared_ptr<Mappoint> Ptr;

    ZJCV_BUILTIN System *mpSystem;

    // Raw data
    ZJCV_BUILTIN parallel::atomic_ptr<std::vector<Observation>> mapObs;

    // Processed data
    ZJCV_BUILTIN float mReprErr = -1;
    ZJCV_BUILTIN Eigen::Vector3f mPos;

    // Frame 创建, Map 清理
    ZJCV_BUILTIN explicit Mappoint(System *pSystem) : mpSystem(pSystem), mapObs(new std::vector<Observation>) {}

    ZJCV_BUILTIN bool is_invalid() const { return mReprErr < 0; }

    ZJCV_BUILTIN void prune();

    ZJCV_BUILTIN void add_obs(const Frame::Ptr &pFrame, const int &idx);

    ZJCV_BUILTIN void clear();

    ZJCV_BUILTIN Mappoint &operator+=(const Mappoint &other);

#ifdef ZJCV_ORB_SLAM
#endif

};

}

}

#endif
