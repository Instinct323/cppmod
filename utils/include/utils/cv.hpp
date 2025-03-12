#ifndef UTILS__CV_HPP
#define UTILS__CV_HPP

#include <eigen3/Eigen/Core>
#include <opencv4/opencv2/opencv.hpp>

#include "glog.hpp"

namespace cv {

// 删除 distance 偏大的匹配
float chi2_filter(std::vector<cv::DMatch> &matches, float chi2, bool ordered = false);

// 使匹配一对一
float make_one2one(std::vector<cv::DMatch> &matches, bool ordered = false);

// Lowe's ratio test
float lowes_filter(const std::vector<std::vector<cv::DMatch>> &knn_matches,
                   std::vector<cv::DMatch> &matches, float ratio);

// 通过余弦相似度过滤匹配
float cosine_filter(const std::vector<Eigen::Vector3f> &unprojs0,
                    const std::vector<Eigen::Vector3f> &unprojs1,
                    std::vector<cv::DMatch> &matches,
                    float thresh);

// dy Pauta Criterion
float dy_filter(const std::vector<Eigen::Vector3f> &unprojs0,
                const std::vector<Eigen::Vector3f> &unprojs1,
                std::vector<cv::DMatch> &matches,
                float sigma_factor,
                float *mean = nullptr);


// Mat -> Eigen
template<typename T>
Eigen::Matrix<T, -1, -1> toEigen(const Mat &mat) {
  Eigen::Matrix<T, -1, -1> m(mat.rows, mat.cols);
  for (int i = 0; i < m.rows(); i++) {
    for (int j = 0; j < m.cols(); j++) {
      m(i, j) = mat.at<T>(i, j);
    }
  }
  return m;
}


// 关键点网格字典
class GridDict {
    int mRows, mCols;
    Size mGridSize;
    Mat_<uchar> mMask;

public:
    explicit GridDict(std::vector<KeyPoint>::iterator begin, std::vector<KeyPoint>::iterator end,
                      const Size &imgSize, const Size &gridSize);

    template<typename T>
    Mat operator()(T x, T y, int dilation = 0) const {
      static_assert(std::is_arithmetic<T>::value, "Invalid type");
      int r = y / mGridSize.height, c = x / mGridSize.width;
      if (!dilation) {
        return mMask.row(r * mCols + c);
      } else {
        Mat_<uchar> ret(1, mMask.cols), tmp = mMask.row(0);
        {
          int cmin = std::max(0, c - dilation), cmax = std::min(mCols - 1, c + dilation) + 1;
          cv::reduce(mMask.rowRange(r * mCols + cmin, r * mCols + cmax), ret, 0, cv::REDUCE_MAX);
        }
        // 纵向膨胀
        for (int d = 1; d <= dilation; d++) {
          int radius = dilation - d;
          int r1 = r - d, r2 = r + d;
          int cmin = std::max(0, c - radius), cmax = std::min(mCols - 1, c + radius) + 1;
          if (r1 >= 0) {
            cv::reduce(mMask.rowRange(r1 * mCols + cmin, r1 * mCols + cmax), tmp, 0, cv::REDUCE_MAX);
            cv::bitwise_or(ret, tmp, ret);
          }
          if (r2 < mRows) {
            cv::reduce(mMask.rowRange(r2 * mCols + cmin, r2 * mCols + cmax), tmp, 0, cv::REDUCE_MAX);
            cv::bitwise_or(ret, tmp, ret);
          }
        }
        return ret;
      }
    }
};


// 关键点水平字典
class HoriDict {
    int mRows;
    Mat_<uchar> mMask;

public:
    explicit HoriDict(std::vector<KeyPoint>::iterator begin,
                      std::vector<KeyPoint>::iterator end,
                      const int &imgRow);

    template<typename T>
    Mat operator()(T y) const {
      static_assert(std::is_arithmetic<T>::value, "Invalid type");
      return mMask.row(int(y));
    }
};


/**
 * @brief 灰度图像加载器
 * @param scale - 图像缩放比例
 * @param clipLimit - 对比度限制
 * @param tileGridSize - 网格大小
 */
class GrayLoader {

public:
    float mScale = 1.f;
    Ptr<CLAHE> mClahe;

    explicit GrayLoader(float scale = 1.f,
                        double clipLimit = 3.0,
                        Size tileGridSize = {8, 8}
    ) : mScale(scale), mClahe(createCLAHE(clipLimit, tileGridSize)) {}

    Mat operator()(const std::string &filename) const {
      Mat img = imread(filename, IMREAD_GRAYSCALE);
      ASSERT(!img.empty(), "fail to load " << filename)
      // 缩放图像, 直方图均衡
      if (mScale != 1.f) resize(img, img, Size(), mScale, mScale);
      mClahe->apply(img, img);
      return img;
    }
};


/**
 * @brief 视频捕获
 * @param src - 视频文件名 / 摄像头编号
 */
class VideoCap : public VideoCapture {

public:
    int mDelay = 0;
    Mat mFrame;

    bool read() {
      bool ret = VideoCapture::read(mFrame);
      // 给定时延, 显示图像
      if (ret && mDelay > 0) {
        imshow("frame", mFrame);
        waitKey(mDelay);
      }
      return ret;
    }

    void reset() { set(CAP_PROP_POS_FRAMES, 0); }

    int length() { return static_cast<int>(get(CAP_PROP_FRAME_COUNT)); }
};


/**
  * @brief 立体深度
  * @param block_matcher - 立体匹配器
  * @param fx - 焦距
  * @param baseline - 基线
  */
class StereoDepth {

public:
    cv::Ptr<cv::StereoSGBM> block_matcher;
    float fx, baseline;

    StereoDepth(
        cv::Ptr<cv::StereoSGBM> block_matcher,
        float fx,
        float baseline
    ) : block_matcher(block_matcher), fx(fx), baseline(baseline) {}

    void to_depth(const cv::Mat &img_left, const cv::Mat &img_right, cv::Mat &depth) {
      // Compute disparity
      cv::Mat disparity;
      block_matcher->compute(img_left, img_right, disparity);
      // FIXME: Compute depth
      float scale = fx * baseline * 5000;
      depth = cv::Mat::zeros(disparity.size(), CV_16UC1);
      cv::parallel_for_(
          cv::Range(0, disparity.rows),
          [&](const cv::Range &range) {
              for (int i = range.start; i < range.end; i++) {
                for (int j = 0; j < disparity.cols; j++) {
                  float d = disparity.at<float>(i, j) / 16.f;
                  depth.at<uint16_t>(i, j) = d > 0 ? scale / d : 0;
                }
              }
          });
    }

};

}

#endif
