#ifndef ZJSLAM__EXTENSION__ORB_HPP
#define ZJSLAM__EXTENSION__ORB_HPP

#include <opencv2/opencv.hpp>

namespace ORB {

typedef std::vector<cv::KeyPoint> KeyPoints;


class Extractor {
    cv::Ptr<cv::ORB> mp;
    std::pair<int, int> mLappingArea;

public:
    Extractor(int nfeatures = 500,
              float scaleFactor = 1.2f,
              int nlevels = 8,
              std::pair<int, int> lappingArea = {-1, -1}
    ) : mp(cv::ORB::create(nfeatures, scaleFactor, nlevels)), mLappingArea(lappingArea) {}

    void operator()(cv::InputArray img, cv::InputArray mask,
                    KeyPoints &keypoints,
                    cv::OutputArray descriptors) {
      mp->detect(img, keypoints, mask);
      // todo: 如果给定重叠区域, 将重叠区域内的关键点移动到末端
      mp->compute(img, keypoints, descriptors);
    }
};

}

#endif
