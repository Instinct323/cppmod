#ifndef UTILS__ORB_HPP
#define UTILS__ORB_HPP

#include <opencv2/opencv.hpp>
#include <utility>
#include <yaml-cpp/yaml.h>

#include "glog.hpp"

namespace ORB {

typedef std::vector<cv::KeyPoint> KeyPoints;


// 特征提取器
class Extractor {
    cv::Ptr<cv::ORB> mpExtractor;
    std::pair<int, int> mLappingArea;

public:
    typedef std::shared_ptr<Extractor> Ptr;

    static Ptr from_yaml(const YAML::Node &cfg);

    Extractor(int nfeatures = 1000,
              float scaleFactor = 1.2f,
              int nlevels = 8,
              std::pair<int, int> lappingArea = {-1, -1}
    ) : mpExtractor(cv::ORB::create(nfeatures, scaleFactor, nlevels)), mLappingArea(std::move(lappingArea)) {
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


class Matcher {
    cv::Ptr<cv::DescriptorMatcher> mpMatcher;

public:
    typedef std::shared_ptr<Matcher> Ptr;

    static Ptr from_yaml(const YAML::Node &cfg);

    explicit Matcher() : mpMatcher(cv::DescriptorMatcher::create("BruteForce-Hamming")) {}

    // common
    void search(const cv::Mat &desc0, const cv::Mat &desc1,
                const cv::Mat &mask, std::vector<cv::DMatch> &matches);

    // Lowe's ratio test
    void search_with_lowe(const cv::Mat &desc0, const cv::Mat &desc1,
                          const cv::Mat &mask, std::vector<cv::DMatch> &matches, float ratio = 0.7);
};

}

#endif
