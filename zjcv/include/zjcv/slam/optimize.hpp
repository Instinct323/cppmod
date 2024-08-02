#ifndef ZJCV__SLAM__OPTIMIZE_HPP
#define ZJCV__SLAM__OPTIMIZE_HPP

#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/types/slam3d/types_slam3d.h>

#include "frame.hpp"
#include "utils/g2o.hpp"

namespace g2o {

// 3D 点投影到深度归一化平面的误差
class EdgeSE3ProjectDNP : public BaseBinaryEdge<2, Eigen::Vector2d, VertexSE3Expmap, VertexPointXYZ> {

public:
    G2O_EMPTY_SERIALIZE

    EdgeSE3ProjectDNP();

    // todo: 把世界点变换到相机坐标系
    void computeError();
};

}

namespace slam::feature {

void optimize_pose(std::shared_ptr<Frame> pFrame);

template<template<typename> class LinearSolverTp>
void bundle_adjustment(std::vector<std::shared_ptr<Frame>> &vpFrames,
                       std::vector<std::weak_ptr<Mappoint>> &vpMappts,
                       bool only_pose);

}

#endif
