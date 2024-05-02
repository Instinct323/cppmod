#ifndef ZJSLAM__UTILS_HPP
#define ZJSLAM__UTILS_HPP

#include <fstream>
#include <opencv2/opencv.hpp>

#define CHECK_FILE(file) if (!file.is_open()) { std::cerr << "Failed to open file: " << #file << std::endl; std::exit(-1); }


// 逐行读出 txt 文件, 并批量映射
void processTxt(const std::string &file, std::function<void(std::string)> unary_op) {
  std::ifstream f(file);
  CHECK_FILE(f)
  // 读取文件, 除去空行和注释
  std::string line;
  while (std::getline(f, line)) {
    if (!line.empty() && line[0] != '#') unary_op(line);
  }
}


// 逐行读出 csv 文件, 并批量映射
void processCsv(const std::string &file, std::function<void(std::vector<std::string>)> unary_op) {
  std::ifstream f(file);
  CHECK_FILE(f)
  // 读取文件, 除去空行和注释
  std::string line;
  while (std::getline(f, line)) {
    if (!line.empty() && line[0] != '#') {
      std::istringstream iss(line);
      std::string token;
      // 以逗号分隔读取
      std::vector<std::string> tokens;
      while (std::getline(iss, token, ',')) tokens.push_back(token);
      unary_op(tokens);
    }
  }
}


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
                         cv::Size tileGridSize = cv::Size(8, 8)
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

    void reset() { set(cv::CAP_PROP_POS_FRAMES, 0); }

    int length() { return static_cast<int>(get(cv::CAP_PROP_FRAME_COUNT)); }
};

#endif
