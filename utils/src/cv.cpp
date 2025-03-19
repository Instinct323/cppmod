#include "utils/cv.hpp"
#include "utils/eigen.hpp"
#include "utils/math.hpp"

namespace cv {


float chi2_filter(std::vector<cv::DMatch> &matches, float chi2, bool ordered) {
    if (matches.empty()) return 0;
    float org = matches.size();

    if (chi2 == 0) return 1;
    if (!ordered) std::sort(matches.begin(), matches.end());

    long double sum_dist = 0;
    for (const cv::DMatch &m: matches) sum_dist += m.distance;
    float thresh = float(sum_dist / matches.size()) * chi2;
    auto it = std::lower_bound(matches.begin(), matches.end(), thresh,
                               [](const cv::DMatch &m, float thresh) { return m.distance < thresh; });
    matches.erase(it, matches.end());
    return float(matches.size()) / org;
}


float make_one2one(std::vector<cv::DMatch> &matches, bool ordered) {
    if (matches.empty()) return 0;
    float org = matches.size();

    if (!ordered) std::sort(matches.begin(), matches.end());
    std::set<int> train;

    auto it = matches.begin();
    while (it != matches.end()) {
        if (train.count(it->trainIdx)) {
            matches.erase(it);
        } else {
            train.insert(it->trainIdx);
            ++it;
        }
    }
    return float(matches.size()) / org;
}


float lowes_filter(const std::vector<std::vector<cv::DMatch>> &knn_matches,
                   std::vector<cv::DMatch> &out_matches, float ratio) {
    ASSERT(out_matches.empty() && ratio > 0 && ratio < 1, "Invalid parameters")
    if (knn_matches.empty()) return 0;
    float org = knn_matches.size();

    out_matches.reserve(knn_matches.size());
    for (const auto &km: knn_matches) {
        if (km.size() == 1 || (km.size() >= 2 && km[0].distance < ratio * km[1].distance)) out_matches.push_back(km[0]);
    }
    return float(out_matches.size()) / org;
}


float cosine_filter(const std::vector<Eigen::Vector3f> &unprojs0,
                    const std::vector<Eigen::Vector3f> &unprojs1,
                    std::vector<cv::DMatch> &matches,
                    float thresh) {
    if (matches.empty()) return 0;
    float org = matches.size();
    // s.t.: ||pi|| = 1, ||\bar{p}|| = 1
    // def: X = Σ xi, Y = Σ yi
    Eigen::Vector2<long double> XY = {0, 0};
    std::vector<Eigen::Vector2f> deltas;
    deltas.reserve(matches.size());
    for (const cv::DMatch &m: matches) {
        Eigen::Vector2f delta = unprojs0[m.queryIdx].head(2) - unprojs1[m.trainIdx].head(2);
        deltas.emplace_back(delta.normalized());
        XY += deltas.back().cast<long double>();
    }
    // maximize: Σ cos(pi, \bar{p}) = \bar{x} X + \bar{y} Y
    // solve: \bar{x} = X / sqrt(X^2 + Y^2), \bar{y} = Y / sqrt(X^2 + Y^2)
    Eigen::Vector2f centroid = XY.normalized().cast<float>();
    for (int i = matches.size() - 1; i >= 0; i--) {
        // 利用余弦相似度筛选 [0, inf)
        float cosine = centroid.dot(deltas[i]);
        if (cosine < thresh) matches.erase(matches.begin() + i);
    }
    return float(matches.size()) / org;
}


float dy_filter(const std::vector<Eigen::Vector3f> &unprojs0,
                const std::vector<Eigen::Vector3f> &unprojs1,
                std::vector<cv::DMatch> &matches,
                float sigma_factor,
                float *mean) {
    if (matches.empty()) return 0;
    float org = matches.size();

    std::vector<float> dy;
    for (cv::DMatch &m: matches) {
        dy.push_back(unprojs0[m.queryIdx][1] - unprojs1[m.trainIdx][1]);
    }
    math::PautaCriterion<float> is_inlier(dy, sigma_factor);
    if (mean) *mean = is_inlier.mMean;

    for (int i = matches.size() - 1; i >= 0; i--) {
        if (!is_inlier(dy[i])) matches.erase(matches.begin() + i);
    }
    return float(matches.size()) / org;
}


GridDict::GridDict(std::vector<KeyPoint>::iterator begin, std::vector<KeyPoint>::iterator end,
                   const Size &imgSize, const Size &gridSize
) : mRows(std::ceil(float(imgSize.height) / gridSize.height)), mCols(std::ceil(float(imgSize.width) / gridSize.width)),
    mGridSize(gridSize), mMask(mRows * mCols, end - begin, uchar(0)) {

    for (auto it = begin; it != end; ++it) {
        float radius = it->size / 2;
        int rMin = std::max(0, int(it->pt.y - radius) / mGridSize.height),
            rMax = std::min(mRows - 1, int(it->pt.y + radius) / mGridSize.height),
            cMin = std::max(0, int(it->pt.x - radius) / mGridSize.width),
            cMax = std::min(mCols - 1, int(it->pt.x + radius) / mGridSize.width);

        for (int r = rMin; r <= rMax; r++) {
            for (int c = cMin; c <= cMax; c++) {
                mMask(r * mCols + c, it - begin) = 1;
            }
        }
    }
}


HoriDict::HoriDict(std::vector<KeyPoint>::iterator begin,
                   std::vector<KeyPoint>::iterator end,
                   const int &imgRow
) : mRows(imgRow), mMask(mRows, end - begin, uchar(0)) {
    for (auto it = begin; it != end; ++it) {
        float y = it->pt.y, r = it->size / 2;
        int col = it - begin;
        int t = std::max(0, int(y - r)), b = std::min(mRows - 1, int(y + r));
        for (int i = t; i <= b; i++) mMask(i, col) = 1;
    }
}

}
