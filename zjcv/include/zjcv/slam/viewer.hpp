#ifndef ZJCV__SLAM__VIEWER_HPP
#define ZJCV__SLAM__VIEWER_HPP

#include <yaml-cpp/yaml.h>

#include "utils/file.hpp"
#include "utils/glog.hpp"
#include "utils/pangolin.hpp"

namespace slam {


class Viewer {

public:
    ZJCV_BUILTIN typedef std::shared_ptr<Viewer> Ptr;

    ZJCV_BUILTIN System *mpSystem;

    ZJCV_CUSTOM explicit Viewer(System *pSystem, const YAML::Node &cfg);

    ZJCV_CUSTOM void run();

#ifdef ZJCV_ORB_SLAM
    int delay;
    int sample_stride;
    int trail_size;
    float imu_size;
    Eigen::Vector3f lead_color, trail_color;
#endif

};

}

#endif
