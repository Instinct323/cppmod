#ifndef ZJCV__SLAM__VIEWER_HPP
#define ZJCV__SLAM__VIEWER_HPP

#include <yaml-cpp/yaml.h>

namespace slam {


class Viewer {

public:
    ZJCV_BUILTIN int delay;
    ZJCV_BUILTIN const float imu_size = 0, mp_size = 0;
    ZJCV_BUILTIN const size_t trail_size = 0;
    ZJCV_BUILTIN const Eigen::Vector3f lead_color, trail_color, mp_color;

    ZJCV_BUILTIN typedef std::shared_ptr<Viewer> Ptr;

    ZJCV_BUILTIN System *mpSystem;

    ZJCV_BUILTIN explicit Viewer(System *pSystem, const YAML::Node &cfg);

    ZJCV_CUSTOM void run();

#ifdef ZJCV_ORB_SLAM
#endif

};

}

#endif
