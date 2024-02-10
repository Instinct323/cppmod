#pragma once

#include "utils.h"


class Frame;


class Mappoint {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<Mappoint> Ptr;
    typedef std::weak_ptr<Frame> FrameWeak;

    static double z_floor;
    Ptr shared_this;

    Vec3 p_w;
    bool is_inlier = true;
    std::vector<std::pair<FrameWeak, int>> kps;  // 关键点 (frame ptr, kp index)

    // 构造方法
    static Ptr create() {
      Ptr p = Ptr(new Mappoint);
      p->shared_this = p;
      return p;
    }

    // 关键点的添加、删除
    void add(const std::shared_ptr<Frame> &ptr, int kp_id) { kps.emplace_back(ptr, kp_id); }

    void reduce() {
      for (int i = kps.size() - 1; i >= 0; --i) {
        auto it = kps.begin() + i;
        if (it->first.expired()) kps.erase(it);
      }
    }

    /**
     * @brief 基于 SVD 的线性三角剖分
     * @param z_floor - 地面高度
     */
    bool triangulation();
};
