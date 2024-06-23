#include "orb.hpp"

namespace ORB {


Extractor::Ptr Extractor::fromYAML(const YAML::Node &node) {
  if (node.IsNull()) return nullptr;
  auto area = YAML::toVec<int>(node["lappingArea"]);
  return Ptr(new Extractor(
      node["nfeatures"].as<int>(),
      node["scaleFactor"].as<float>(),
      node["nlevels"].as<int>(),
      {area[0], area[1]}
  ));
}


int Extractor::detectAndCompute(cv::InputArray img, cv::InputArray mask,
                                KeyPoints &keypoints, cv::OutputArray descriptors) {
  mpExtractor->detectAndCompute(img, mask, keypoints, descriptors);
  int monoCnt = 0;
  if (mLappingArea.first >= 0) {
    // 全部在重叠区域内
    if (mLappingArea.first == 0 && mLappingArea.second < img.cols()) {
      return keypoints.size();
    }
    // 重叠区域的点聚集到前面
    cv::Mat desc = descriptors.getMat();
    for (int i = 0; i < keypoints.size(); i++) {
      if (mLappingArea.first <= keypoints[i].pt.x && keypoints[i].pt.x <= mLappingArea.second) {
        if (monoCnt != i) {
          std::swap(keypoints[monoCnt], keypoints[i]);
          // 交换描述子
          cv::Mat tmp = desc.row(monoCnt).clone();
          desc.row(i).copyTo(desc.row(monoCnt));
          tmp.copyTo(desc.row(i));
        }
        monoCnt++;
      }
    }
  }
  return monoCnt;
}

}
