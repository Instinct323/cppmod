#include "fbow.hpp"

namespace fbow {

void createORBvocabulary(Vocabulary &voc,
                         ORB::Extractor::Ptr extractor,
                         cv::GrayLoader &loader,
                         std::vector<std::string> &filenames,
                         Params params) {
  // 提取 ORB 描述子
  std::vector<cv::Mat> descriptors(filenames.size(), cv::Mat());
  // todo: 增加进度条, 线程池
  for (int i = 0; i < filenames.size(); i++) {
    ORB::KeyPoints kps;
    extractor->detectAndCompute(loader(filenames[i]), cv::noArray(), kps, descriptors[i]);
  }
  // 训练词汇表
  VocabularyCreator creator;
  creator.create(voc, descriptors, "ORB", params);
}

}
