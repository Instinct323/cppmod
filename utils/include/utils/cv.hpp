#ifndef UTILS__CV_HPP
#define UTILS__CV_HPP

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "glog.hpp"

namespace cv {

// 删除 distance 偏大的匹配
void drop_last(std::vector<cv::DMatch> &matches, float radio);

// 使匹配一对一
void make_one2one(std::vector<cv::DMatch> &matches);

// Lowe's ratio test
void lowes_filter(const std::vector<std::vector<cv::DMatch>> &knn_matches,
                  std::vector<cv::DMatch> &matches, float ratio);

// 通过余弦相似度过滤匹配
void cosine_filter(const std::vector<Eigen::Vector3f> &unprojs0,
                   const std::vector<Eigen::Vector3f> &unprojs1,
                   std::vector<cv::DMatch> &matches,
                   float sigma_factor);

// dy Pauta Criterion
void dy_filter(const std::vector<Eigen::Vector3f> &unprojs0,
               const std::vector<Eigen::Vector3f> &unprojs1,
               std::vector<cv::DMatch> &matches,
               float sigma_factor);


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
                      const Size &imgSize, const Size &gridSize = {16, 16}, int dilation = 1);

    template<typename T>
    Mat operator()(T x, T y) const {
      static_assert(std::is_arithmetic<T>::value, "Invalid type");
      int r = y / mGridSize.height, c = x / mGridSize.width;
      return mMask.row(r * mCols + c);
    }
};


// 关键点水平字典
class HoriDict {
    int mRows;
    int mStride;
    Mat_<uchar> mMask;

public:
    explicit HoriDict(std::vector<KeyPoint>::iterator begin, std::vector<KeyPoint>::iterator end,
                      const int &imgRow, const int &stride);

    template<typename T>
    Mat operator()(T y) const {
      static_assert(std::is_arithmetic<T>::value, "Invalid type");
      return mMask.row(y / mStride);
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

}

#endif
