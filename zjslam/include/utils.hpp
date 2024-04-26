#ifndef ZJSLAM__UTILS_HPP
#define ZJSLAM__UTILS_HPP

#include <opencv2/opencv.hpp>


/**
 * @brief 视频捕获
 * @param src - 视频文件名 / 摄像头编号
 */
class VideoCap : public cv::VideoCapture {

public:
    int delay = 0;
    cv::Mat frame;

    bool read() {
      bool ret = cv::VideoCapture::read(frame);
      // 给定时延, 显示图像
      if (ret && delay > 0) {
        cv::imshow("frame", frame);
        cv::waitKey(delay);
      }
      return ret;
    }

    void reset() {
      set(cv::CAP_PROP_POS_FRAMES, 0);
    }

    int length() {
      return static_cast<int>(get(cv::CAP_PROP_FRAME_COUNT));
    }
};

#endif
