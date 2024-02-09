#pragma once

#include "utils.h"


class Frame;


class Mappoint {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Mappoint> Ptr;
    typedef std::weak_ptr<Frame> FramePtr;

    Vec3 pos;
    std::vector<std::pair<FramePtr, int>> kps;  // 关键点 (frame ptr, kp index)

    Mappoint(Vec3 pos) : pos(pos) {}

    // 关键点的添加、删除
    void add(FramePtr ptr, int kp_id) { kps.push_back({ptr, kp_id}); }

    void reduce() {
      for (int i = kps.size() - 1; i >= 0; --i) {
        auto it = kps.begin() + i;
        if (it->first.expired()) kps.erase(it);
      }
    }

    // 优化当前路标点
    void optimize() {}
};
