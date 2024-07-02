#ifndef UTILS__ORB_HPP
#define UTILS__ORB_HPP

#include <opencv2/opencv.hpp>

#include "file.hpp"
#include "glog.hpp"

namespace ORB {

typedef std::vector<cv::KeyPoint> KeyPoints;


// 亚像素级精化 (模板匹配)
bool matchesSubPix(const cv::Mat &img0, const cv::Mat &img1,
                   const cv::Point2f &kp0, cv::Point2f &kp1,
                   int winSize = 5, cv::Size slideSize = {5, 5});


// Lowe's ratio test
void lowes_filter(const std::vector<std::vector<cv::DMatch>> &knn_matches,
                  std::vector<cv::DMatch> &matches, float ratio);


// 特征提取器
class Extractor {
    cv::Ptr<cv::ORB> mpExtractor;
    std::pair<int, int> mLappingArea;

public:
    typedef std::shared_ptr<Extractor> Ptr;

    static Ptr from_yaml(const YAML::Node &node);

    Extractor(int nfeatures = 1000,
              float scaleFactor = 1.2f,
              int nlevels = 8,
              std::pair<int, int> lappingArea = {-1, -1}
    ) : mpExtractor(cv::ORB::create(nfeatures, scaleFactor, nlevels)), mLappingArea(lappingArea) {
      ASSERT((mLappingArea.first == -1 && mLappingArea.second == -1) ||
             (0 <= mLappingArea.first < mLappingArea.second), "Invalid lapping area")
    }

    /**
     * @brief ORB 关键点提取及描述子计算
     * @return 重叠区域内的关键点数量
     */
    int detect_and_compute(cv::InputArray img, cv::InputArray mask,
                           KeyPoints &keypoints, cv::OutputArray &desc);
};

}

#endif
