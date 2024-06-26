#include "utils/orb.hpp"

namespace ORB {


bool matchesSubPix(const cv::Mat &img0, const cv::Mat &img1,
                   const cv::Point2f &kp0, cv::Point2f &kp1,
                   int winSize, cv::Size slideSize) {
  ASSERT(winSize & 1 && winSize > 1 && slideSize.width > 0 && slideSize.height > 0, "Invalid parameters")
  cv::Rect roi0(cvRound(kp0.x) - slideSize.width / 2, cvRound(kp0.y) - slideSize.height / 2, slideSize.width, slideSize.height);
  if (roi0.x < 0 || roi0.y < 0 || roi0.x + roi0.width >= img0.cols || roi0.y + roi0.height >= img0.rows) return false;
  // 模板尺寸相关参数
  cv::Size winSizeR(slideSize.width + winSize - 1, slideSize.height + winSize - 1);
  cv::Rect roi1(cvRound(kp1.x) - winSizeR.width / 2, cvRound(kp1.y) - winSizeR.height / 2, winSizeR.width, winSizeR.height);
  if (roi1.x < 0 || roi1.y < 0 || roi1.x + roi1.width >= img1.cols || roi1.y + roi1.height >= img1.rows) return false;
  int xBias = slideSize.width / 2, yBias = slideSize.height / 2;
  // 模板匹配
  cv::Mat roiImg0 = img0(roi0), roiImg1 = img1(roi1), result;
  cv::matchTemplate(roiImg1, roiImg0, result, cv::TM_SQDIFF_NORMED);
  cv::Point2i bestLoc;
  cv::minMaxLoc(result, nullptr, nullptr, &bestLoc, nullptr);
  // x 抛物线拟合
  if (bestLoc.x > 0 && bestLoc.x < slideSize.width - 1) {
    float v0 = result.at<float>(bestLoc.y, bestLoc.x - 1),
        v1 = result.at<float>(bestLoc.y, bestLoc.x),
        v2 = result.at<float>(bestLoc.y, bestLoc.x + 1);
    float delta = (v0 - v2) / (v0 + v2 - 2 * v1);
    if (std::abs(delta) < 1) {
      kp1.x += float(bestLoc.x - xBias) + delta;
      bestLoc.x = cvRound(float(bestLoc.x) + delta);
      cv::minMaxLoc(result.col(bestLoc.x), nullptr, nullptr, &bestLoc, nullptr);
    }
  }
  // y 抛物线拟合
  if (bestLoc.y > 0 && bestLoc.y < slideSize.height - 1) {
    float v0 = result.at<float>(bestLoc.y - 1, bestLoc.x),
        v1 = result.at<float>(bestLoc.y, bestLoc.x),
        v2 = result.at<float>(bestLoc.y + 1, bestLoc.x);
    float delta = (v0 - v2) / (v0 + v2 - 2 * v1);
    if (std::abs(delta) < 1) kp1.y += float(bestLoc.y - yBias) + delta;
  }
  return true;
}


void lowes_filter(const std::vector<std::vector<cv::DMatch>> &knn_matches,
                  std::vector<cv::DMatch> &matches, float ratio) {
  ASSERT(matches.empty() && ratio > 0 && ratio < 1, "Invalid parameters")
  for (const auto &km: knn_matches) {
    if (km.size() == 1 || (km.size() >= 2 && km[0].distance < ratio * km[1].distance)) matches.push_back(km[0]);
  }
}


Extractor::Ptr Extractor::from_yaml(const YAML::Node &node) {
  if (node.IsNull()) return nullptr;
  auto area = YAML::toVec<int>(node["lappingArea"]);
  return Ptr(new Extractor(
      node["nfeatures"].as<int>(),
      node["scaleFactor"].as<float>(),
      node["nlevels"].as<int>(),
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

}
