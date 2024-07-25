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

typedef std::pair<Frame *, int> Observation;


class Mappoint {

public:
    ZJCV_BUILTIN typedef std::shared_ptr<Mappoint> Ptr;

    ZJCV_BUILTIN Eigen::Vector3f mPos;
    ZJCV_BUILTIN std::vector<Observation> mObs;

    ZJCV_BUILTIN int frame_count() const {
      std::set<Frame *> frames;
      for (auto &obs: mObs) frames.insert(obs.first);
      return frames.size();
    }

    ZJCV_BUILTIN void add_obs(Frame *pFrame, const int &idx);

    ZJCV_BUILTIN void erase_obs(Frame *pFrame);

    ZJCV_BUILTIN Mappoint& operator+=(const Mappoint& other);

    ZJCV_CUSTOM void process();

#ifdef ZJCV_ORB_SLAM
#endif

};

}

}

#endif
