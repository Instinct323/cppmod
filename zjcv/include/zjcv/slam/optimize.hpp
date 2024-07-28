#ifndef ZJCV__SLAM__OPTIMIZE_HPP
#define ZJCV__SLAM__OPTIMIZE_HPP

#include "frame.hpp"
#include "utils/g2o.hpp"

namespace slam::feature {

void optimize_pose(Frame *pFrame);

}

#endif
