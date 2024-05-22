#ifndef ZJSLAM__EXTENSION__ORB_HPP
#define ZJSLAM__EXTENSION__ORB_HPP

#include <opencv2/opencv.hpp>

#include "../file.hpp"

namespace ORB {

typedef std::vector<cv::KeyPoint> KeyPoints;


class Extractor {
    cv::Ptr<cv::ORB> mpExtractor;
    std::pair<int, int> mLappingArea;

public:
    typedef std::shared_ptr<Extractor> Ptr;

    static Ptr fromYAML(const YAML::Node &node) {
      auto area = YAML::toVec<int>(node["lappingArea"]);
      return Ptr(new Extractor(
          node["nfeatures"].as<int>(),
          node["scaleFactor"].as<float>(),
          node["nlevels"].as<int>(),
          {area[0], area[1]}
      ));
    }

    Extractor(int nfeatures = 1500,
              float scaleFactor = 1.2f,
              int nlevels = 8,
              std::pair<int, int> lappingArea = {-1, -1}
    ) : mpExtractor(cv::ORB::create(nfeatures, scaleFactor, nlevels)), mLappingArea(lappingArea) {
      ASSERT((mLappingArea.first == -1 && mLappingArea.second == -1) ||
             (0 <= mLappingArea.first < mLappingArea.second), "Invalid lapping area");
    }

    /**
     * @brief ORB 关键点提取及描述子计算
     * @return 重叠区域内的关键点数量
     */
    int operator()(cv::InputArray img, cv::InputArray mask,
                   KeyPoints &keypoints,
                   cv::OutputArray descriptors);
};


int Extractor::operator()(cv::InputArray img, cv::InputArray mask,
                          KeyPoints &keypoints,
                          cv::OutputArray descriptors) {
  mpExtractor->detectAndCompute(img, mask, keypoints, descriptors);
  int monoCnt = 0;
  // 重叠区域的点 聚集到前面
  if (mLappingArea.first >= 0) {
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

#endif
