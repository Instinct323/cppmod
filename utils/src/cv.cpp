#include "utils/cv.hpp"
#include "utils/math.hpp"

namespace cv {


void drop_last(std::vector<cv::DMatch> &matches, float radio) {
  if (radio == 0) return;
  int n = float(matches.size()) * radio;
  std::sort(matches.begin(), matches.end());
  matches.erase(matches.end() - n, matches.end());
}


void make_one2one(std::vector<cv::DMatch> &matches) {
  std::sort(matches.begin(), matches.end());
  std::set<int> query, train;

  auto it = matches.begin();
  while (it != matches.end()) {
    if (query.count(it->queryIdx) || train.count(it->trainIdx)) {
      matches.erase(it);
    } else {
      query.insert(it->queryIdx);
      train.insert(it->trainIdx);
      it++;
    }
  }
}


void lowes_filter(const std::vector<std::vector<cv::DMatch>> &knn_matches,
                  std::vector<cv::DMatch> &out_matches, float ratio) {
  ASSERT(out_matches.empty() && ratio > 0 && ratio < 1, "Invalid parameters")
  out_matches.reserve(knn_matches.size());
  for (const auto &km: knn_matches) {
    if (km.size() == 1 || (km.size() >= 2 && km[0].distance < ratio * km[1].distance)) out_matches.push_back(km[0]);
  }
}


void cosine_filter(const std::vector<Eigen::Vector3f> &unprojs0,
                   const std::vector<Eigen::Vector3f> &unprojs1,
                   std::vector<cv::DMatch> &matches,
                   float sigma_factor) {
  // s.t.: ||pi|| = 1, ||\bar{p}|| = 1
  // def: X = Σ xi, Y = Σ yi
  Eigen::Vector2<long double> XY = {0, 0};
  std::vector<Eigen::Vector2f> deltas;
  deltas.reserve(matches.size());
  for (const cv::DMatch &m: matches) {
    Eigen::Vector2f delta = unprojs0[m.queryIdx].head(2) - unprojs1[m.trainIdx].head(2);
    deltas.emplace_back(delta / delta.norm());
    XY += deltas.back().cast<long double>();
  }
  // maximize: Σ cos(pi, \bar{p}) = \bar{x} X + \bar{y} Y
  // solve: \bar{x} = X / sqrt(X^2 + Y^2), \bar{y} = Y / sqrt(X^2 + Y^2)
  Eigen::Vector2f centroid = XY.normalized().cast<float>();
  // 利用余弦相似度筛选 [0, inf)
  std::vector<float> cosines;
  cosines.reserve(deltas.size());
  for (const Eigen::Vector2f &delta: deltas) cosines.push_back(1 - centroid.dot(delta));
  // 根据标准差计算阈值
  float thresh = sqrt(math::mean_square(cosines)) * sigma_factor;
  for (int i = matches.size() - 1; i >= 0; i--) {
    if (cosines[i] > thresh) matches.erase(matches.begin() + i);
  }
}


void dy_filter(const std::vector<Eigen::Vector3f> &unprojs0,
               const std::vector<Eigen::Vector3f> &unprojs1,
               std::vector<cv::DMatch> &matches,
               float sigma_factor) {
  std::vector<float> dy;
  for (cv::DMatch &m: matches) {
    dy.push_back(unprojs0[m.queryIdx][1] - unprojs1[m.trainIdx][1]);
  }
  math::PautaCriterion<float> is_inlier(dy, sigma_factor);
  for (int i = matches.size() - 1; i >= 0; i--) {
    if (!is_inlier(dy[i])) matches.erase(matches.begin() + i);
  }
}


GridDict::GridDict(std::vector<KeyPoint>::iterator begin, std::vector<KeyPoint>::iterator end,
                   const Size &imgSize, const Size &gridSize, int dilation
) : mRows(std::ceil(imgSize.height / gridSize.height)), mCols(std::ceil(imgSize.width / gridSize.width)),
    mGridSize(gridSize), mMask(mRows * mCols, end - begin, uchar(0)) {
  assert(dilation >= 0);

  for (auto it = begin; it != end; it++) {
    float radius = it->size / 2;
    int rMin = std::max(0, int(it->pt.y - radius) / mGridSize.height - dilation),
        rMax = std::min(mRows, int(it->pt.y + radius) / mGridSize.height + dilation),
        cMin = std::max(0, int(it->pt.x - radius) / mGridSize.width - dilation),
        cMax = std::min(mCols - 1, int(it->pt.x + radius) / mGridSize.width + dilation);

    for (int r = rMin; r <= rMax; r++) {
      for (int c = cMin; c <= cMax; c++) {
        mMask(r * mCols + c, it - begin) = 1;
      }
    }
  }
}


HoriDict::HoriDict(std::vector<KeyPoint>::iterator begin, std::vector<KeyPoint>::iterator end,
                   const int &imgRow, const int &stride
) : mRows(std::ceil(imgRow / stride)), mStride(stride),
    mMask(mRows, end - begin, uchar(0)) {
  for (auto it = begin; it != end; it++) {
    float y = it->pt.y, r = it->size / 2;
    int t = std::max(0, int(y - r) / mStride), b = std::min(mRows - 1, int(y + r) / mStride);
    for (int i = t; i <= b; i++) mMask(i, it - begin) = 1;
  }
}

}
