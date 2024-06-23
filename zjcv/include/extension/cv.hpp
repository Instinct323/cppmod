#ifndef ZJCV__EXTENSION__CV_HPP
#define ZJCV__EXTENSION__CV_HPP

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

namespace cv {


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

    Mat operator()(const std::string &filename) const;
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
