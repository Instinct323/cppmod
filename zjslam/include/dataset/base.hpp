#ifndef ZJSLAM__DATASET__BASE_HPP
#define ZJSLAM__DATASET__BASE_HPP

#include <filesystem>
#include <fstream>
#include <sophus/se3.hpp>


void processTxt(const std::string &file, std::function<void(std::string)> unary_op) {
  std::ifstream f(file);
  // 校验文件打开状态
  if (!f.is_open()) {
    std::cerr << "Failed to open file: " << file << std::endl;
    std::exit(-1);
  }
  // 读取文件, 除去空行和注释
  std::string line;
  while (std::getline(f, line)) {
    if (!line.empty() && line[0] != '#') unary_op(line);
  }
}


class DatasetBase {

protected:
    std::string mPath;

public:
    typedef std::vector<double> Timestamps;
    typedef std::vector<std::string> Filenames;
    typedef std::vector<Sophus::SE3d> Poses;
    typedef std::vector<Eigen::Vector3d> Accels;

    DatasetBase(const std::string &path) : mPath(path + "/") {}

    friend std::ostream &operator<<(std::ostream &os, const DatasetBase &dataset) {
      return os << "DatasetBase(" << dataset.mPath << ")";
    }
};

#endif
