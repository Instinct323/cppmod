#include <memory>

#include "utils/cv.hpp"
#include "utils/file.hpp"
#include "utils/orb.hpp"

namespace ORB {


// Extractor
Extractor::Ptr Extractor::from_yaml(const YAML::Node &cfg) {
  if (YAML::is_invalid(cfg)) return nullptr;
  auto area = YAML::toVec<int>(cfg["lapping_area"]);
  return Ptr(new Extractor(
      cfg["nfeatures"].as<int>(),
      cfg["scale_factor"].as<float>(),
      cfg["nlevels"].as<int>(),
      {area[0], area[1]}
  ));
}


int Extractor::detect_and_compute(cv::InputArray img, cv::InputArray mask,
                                  KeyPoints &keypoints, cv::OutputArray &desc) {
  mpExtractor->detectAndCompute(img, mask, keypoints, desc);
  int lapCnt = 0;
  if (mLappingArea.first >= 0) {
    // 全部在重叠区域内
    if (mLappingArea.first == 0 && mLappingArea.second < img.cols()) {
      return keypoints.size();
    }
    // 重叠区域的点聚集到前面
    cv::Mat descriptors = desc.getMat();
    for (int i = 0; i < keypoints.size(); i++) {
      if (mLappingArea.first <= keypoints[i].pt.x && keypoints[i].pt.x <= mLappingArea.second) {
        if (lapCnt != i) {
          std::swap(keypoints[lapCnt], keypoints[i]);
          // 交换描述子
          cv::Mat tmp = descriptors.row(lapCnt).clone();
          descriptors.row(i).copyTo(descriptors.row(lapCnt));
          tmp.copyTo(descriptors.row(i));
        }
        lapCnt++;
      }
    }
  }
  return lapCnt;
}


// Matcher
Matcher::Ptr Matcher::from_yaml(const YAML::Node &cfg) {
  if (YAML::is_invalid(cfg)) return nullptr;
  return Ptr(new Matcher());
}


void Matcher::search(const cv::Mat &desc0, const cv::Mat &desc1,
                     const cv::Mat &mask, std::vector<cv::DMatch> &matches) {
  mpMatcher->match(desc0, desc1, matches, mask);
}


void Matcher::search_with_lowe(const cv::Mat &desc0, const cv::Mat &desc1,
                               const cv::Mat &mask, std::vector<cv::DMatch> &matches, float ratio) {
  std::vector<std::vector<cv::DMatch>> knn_matches;
  mpMatcher->knnMatch(desc0, desc1, knn_matches, 2, mask);
  cv::lowes_filter(knn_matches, matches, ratio);
}

}
