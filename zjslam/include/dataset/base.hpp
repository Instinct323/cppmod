#ifndef ZJSLAM__DATASET__BASE_HPP
#define ZJSLAM__DATASET__BASE_HPP

#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>


bool processTxt(const std::string &file, std::function<void(std::string)> unary_op) {
  std::ifstream f(file);
  // 校验文件打开状态
  if (!f.is_open()) {
    std::cerr << "Failed to open file: " << file << std::endl;
    return false;
  }
  // 读取文件, 除去空行和注释
  std::string line;
  while (std::getline(f, line)) {
    if (!line.empty() && line[0] != '#') unary_op(line);
  }
  return true;
}


class DatasetBase {

protected:
    std::string path;

public:
    typedef std::vector<double> Timestamps;
    typedef std::vector<std::string> Filenames;
    typedef std::vector<Sophus::SE3d> Poses;
    typedef std::vector<Eigen::Vector3d> Accels;

    DatasetBase(const std::string &path) : path(path + "/") {}

    friend std::ostream &operator<<(std::ostream &os, const DatasetBase &dataset) {
      return os << "DatasetBase(" << dataset.path << ")";
    }
};


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
