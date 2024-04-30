#ifndef ZJSLAM__DATA_HPP
#define ZJSLAM__DATA_HPP

#include <fstream>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

// todo: KITTI https://www.cvlibs.net/datasets/kitti/eval_odometry.php


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


class Dataset {

protected:
    std::string path;

public:
    typedef std::vector<double> Timestamps;
    typedef std::vector<std::string> ImgFiles;
    typedef std::vector<Sophus::SE3d> Poses;
    typedef std::vector<Eigen::Vector3d> Accels;

    Dataset(const std::string &path) : path(path) {}

    friend std::ostream &operator<<(std::ostream &os, const Dataset &dataset) {
      return os << "Dataset(" << dataset.path << ")";
    }
};


/**
 * @brief TUM-RGBD https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download
 */
class TumRgbd : public Dataset {

public:
    TumRgbd(const std::string &path) : Dataset(path) {}

    // rgb.txt, depth.txt
    void loadImage(Timestamps &vTimestamps, ImgFiles &vFilename, std::string file = "/rgb.txt") {
      processTxt(path + file,
                 [&vTimestamps, &vFilename](std::string line) {
                     std::istringstream iss(line);
                     double timestamp;
                     std::string filename;
                     // timestamp filename
                     iss >> timestamp >> filename;
                     vTimestamps.push_back(timestamp);
                     vFilename.push_back(filename);
                 });
    }

    // groundtruth.txt
    void loadPoses(Timestamps &vTimestamps, Poses &vPoses, std::string file = "/groundtruth.txt") {
      processTxt(path + file,
                 [&vTimestamps, &vPoses](std::string line) {
                     std::istringstream iss(line);
                     double timestamp;
                     double tx, ty, tz, qx, qy, qz, qw;
                     // timestamp tx ty tz qx qy qz qw
                     iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
                     vTimestamps.push_back(timestamp);
                     vPoses.emplace_back(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
                 });
    }

    // accelerometer.txt
    void loadAccel(Timestamps &vTimestamps, Accels &vAccel, std::string file = "/accelerometer.txt") {
      processTxt(path + file,
                 [&vTimestamps, &vAccel](std::string line) {
                     std::istringstream iss(line);
                     double timestamp;
                     double ax, ay, az;
                     // timestamp ax ay az
                     iss >> timestamp >> ax >> ay >> az;
                     vTimestamps.push_back(timestamp);
                     vAccel.emplace_back(ax, ay, az);
                 });
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
