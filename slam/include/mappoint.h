#pragma once

#include <memory>

#include "utils.h"


class Frame;


class Mappoint {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<Mappoint> Ptr;
    typedef std::weak_ptr<Frame> FrameWeak;

    static std::vector<std::weak_ptr<Mappoint>> map;
    static double z_floor;

    Vec3 p_w;
    bool is_inlier = false;
    std::vector<std::pair<FrameWeak, int>> kps;  // 关键点 (frame ptr, kp index)

    static Ptr create() {
      Ptr p = std::make_shared<Mappoint>();
      map.push_back(p);
      return p;
    }

    // 关键点的添加
    void add(const FrameWeak &ptr, int kp_id) { kps.emplace_back(ptr, kp_id); }

    // 基于 SVD 的线性三角剖分
    void triangulation();

protected:
    friend std::ostream &operator<<(std::ostream &os, const Mappoint &mp) {
      format fmt;
      if (mp.is_inlier) {
        fmt = format("Mappoint(%.2f, %.2f, %.2f, n_kps=%d)");
        fmt % mp.p_w[0] % mp.p_w[1] % mp.p_w[2];
      } else {
        fmt = format("Mappoint(n_kps=%d)");
      }
      return os << (fmt % mp.kps.size());
    }
};
