#include <fstream>
#include <sophus/se3.hpp>
#include <yaml-cpp/yaml.h>

#include "utils/eigen.hpp"
#include "utils/file.hpp"
#include "utils/glog.hpp"

namespace CSV {

// 逐行映射
void row_mapping(const std::string &file, const std::function<void(std::vector<std::string> &)> &unary_op) {
  std::ifstream f(file);
  ASSERT(f.is_open(), "fail to open file " << file)
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

}


namespace TXT {

// 逐行映射
void row_mapping(const std::string &file, const std::function<void(std::string &)> &unary_op) {
  std::ifstream f(file);
  ASSERT(f.is_open(), "fail to open file " << file)
  // 读取文件, 除去空行和注释
  std::string line;
  while (std::getline(f, line)) {
    if (!line.empty() && line[0] != '#') unary_op(line);
  }
}

}


namespace YAML {

bool is_invalid(const Node &node) {
  try {
    return node.IsNull();
  } catch (YAML::InvalidNode &e) {
    return true;
  }
}


void assert_matrix(const Node &node) {
  bool flag = node.IsSequence();
  if (flag) {
    // 校对每一行的列数
    size_t rows = node.size();
    size_t cols = node[0].size();
    if (cols) {
      for (int i = 0; i < rows; ++i) {
        flag = node[i].IsSequence() && node[i].size() == cols;
        if (!flag) break;
      }
    }
  }
  if (!flag) LOG(FATAL) << "YAML: Invalid matrix format";
}

}
