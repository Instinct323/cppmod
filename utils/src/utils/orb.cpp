#include "utils/orb.hpp"

namespace ORB {


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
  int monoCnt = 0;
  if (mLappingArea.first >= 0) {
    // 全部在重叠区域内
    if (mLappingArea.first == 0 && mLappingArea.second < img.cols()) {
      return keypoints.size();
    }
    // 重叠区域的点聚集到前面
    cv::Mat descriptors = desc.getMat();
    for (int i = 0; i < keypoints.size(); i++) {
      if (mLappingArea.first <= keypoints[i].pt.x && keypoints[i].pt.x <= mLappingArea.second) {
        if (monoCnt != i) {
          std::swap(keypoints[monoCnt], keypoints[i]);
          // 交换描述子
          cv::Mat tmp = descriptors.row(monoCnt).clone();
          descriptors.row(i).copyTo(descriptors.row(monoCnt));
          tmp.copyTo(descriptors.row(i));
        }
        monoCnt++;
      }
    }
  }
  return monoCnt;
}

}
