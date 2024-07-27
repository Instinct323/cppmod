#ifndef ZJCV__SLAM__MAPPOINT_HPP
#define ZJCV__SLAM__MAPPOINT_HPP

#include <map>
#include <opencv2/opencv.hpp>
#include <set>
#include <sophus/se3.hpp>

#include "zjcv/zjcv.hpp"

namespace slam {

// 基于特征点
namespace feature {

class Frame;

typedef std::pair<std::weak_ptr<Frame>, int> Observation;


class Mappoint {

public:
    ZJCV_BUILTIN typedef std::shared_ptr<Mappoint> Ptr;

    ZJCV_BUILTIN System *mpSystem;

    ZJCV_BUILTIN float mReprErr = -1;
    ZJCV_BUILTIN Eigen::Vector3f mPos;
    ZJCV_BUILTIN std::vector<Observation> mObs;

    ZJCV_BUILTIN explicit Mappoint(System *pSystem) : mpSystem(pSystem) {}

    ZJCV_BUILTIN bool is_invalid() const { return mReprErr < 0; }

    ZJCV_BUILTIN int frame_count();

    ZJCV_BUILTIN void prune();

    ZJCV_BUILTIN void add_obs(const Frame::Ptr& pFrame, const int &idx, bool is_right = false);

    ZJCV_BUILTIN void clear();

    ZJCV_BUILTIN Mappoint &operator+=(const Mappoint &other);

    ZJCV_BUILTIN void triangulation();

#ifdef ZJCV_ORB_SLAM
#endif

};

}

}

#endif
