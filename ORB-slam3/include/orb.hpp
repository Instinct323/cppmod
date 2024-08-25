#ifndef MY_ORB_HPP
#define MY_ORB_HPP

#include "utils/cv.hpp"
#include "zjcv/slam.hpp"

namespace ORB {

bool search_by_project(slam::feature::Frame* pRef, slam::feature::Frame* pCur, std::vector<cv::DMatch> &matches);

}

#endif
