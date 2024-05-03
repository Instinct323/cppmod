#ifndef ZJSLAM__UTILS_HPP
#define ZJSLAM__UTILS_HPP

#include <fstream>
#include <opencv2/opencv.hpp>

#include "logging.hpp"


/**
 * @brief 按值切片器
 */
template<typename T>
class ValueSlicer {
    int mIndex = 0;
    const std::vector<T> mValues;
    std::function<bool(const T &, const T &)> mCompare;

public:
    explicit ValueSlicer(const std::vector<T> &values, bool ascending = true
    ) : mValues(values), mCompare(ascending ? [](const T &a, const T &b) { return a <= b; } :
                                  [](const T &a, const T &b) { return a >= b; }) {}

    // 返回切片索引
    std::pair<int, int> operator()(T value) {
      ASSERT(mIndex < mValues.size(), "ValueSlicer: The slicer has expired");
      ASSERT(mCompare(mValues[mIndex], value), "ValueSlicer: Invalid input value");
      int i = mIndex;
      for (; mIndex < mValues.size(); ++mIndex) {
        if (!mCompare(mValues[mIndex], value)) break;
      }
      return {i, mIndex};
    }
};


/**
 * @brief 图像加载器
 * @param scale - 图像缩放比例
 * @param clipLimit - 对比度限制
 * @param tileGridSize - 网格大小
 */
class ImageLoader {
public:
    float mScale = 1.f;
    cv::Ptr<cv::CLAHE> mClahe;

    explicit ImageLoader(float scale = 1.f,
                         double clipLimit = 3.0,
                         cv::Size tileGridSize = {8, 8}
    ) : mScale(scale), mClahe(cv::createCLAHE(clipLimit, tileGridSize)) {}

    cv::Mat operator()(const std::string &filename) const {
      cv::Mat img = cv::imread(filename, cv::IMREAD_GRAYSCALE);
      // 检查图像状态
      if (img.empty()) {
        std::cerr << "fail to load " << filename << std::endl;
        std::exit(-1);
      }
      // 缩放图像, 直方图均衡
      if (mScale != 1.f) cv::resize(img, img, cv::Size(), mScale, mScale);
      mClahe->apply(img, img);
      return img;
    }
};


/**
 * @brief 视频捕获
 * @param src - 视频文件名 / 摄像头编号
 */
class VideoCap : public cv::VideoCapture {

public:
    int mDelay = 0;
    cv::Mat mFrame;

    bool read() {
      bool ret = cv::VideoCapture::read(mFrame);
      // 给定时延, 显示图像
      if (ret && mDelay > 0) {
        cv::imshow("frame", mFrame);
        cv::waitKey(mDelay);
      }
      return ret;
    }

    void reset() { set(cv::CAP_PROP_POS_FRAMES, 0); }

    int length() { return static_cast<int>(get(cv::CAP_PROP_FRAME_COUNT)); }
};

#endif
