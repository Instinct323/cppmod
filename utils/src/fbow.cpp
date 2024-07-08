#include "utils/fbow.hpp"
#include "utils/indicators.hpp"
#include "utils/parallel.hpp"

namespace fbow {

void createORBvocabulary(Vocabulary &voc,
                         ORB::Extractor *extractor,
                         cv::GrayLoader &loader,
                         std::vector<std::string> &filenames,
                         Params params) {
  std::vector<cv::Mat> descriptors(filenames.size(), cv::Mat());
  // 多线程提取 ORB 描述子
  auto pbar = indicators::getProgressBar(filenames.size());
  indicators::set_desc(pbar, "Extracting ORB descriptors");
  for (int i = 0; i < filenames.size(); i++) {
    ORB::KeyPoints kps;
    parallel::thread_pool.emplace(1, std::bind(
        &ORB::Extractor::detect_and_compute, extractor, loader(filenames[i]), cv::Mat(), kps, std::ref(descriptors[i])));
    pbar.tick();
  }
  parallel::thread_pool.join(1);
  for (int i = 0; i < filenames.size(); i++) std::cout << descriptors[i].size() << " ";
  // 训练词汇表
  LOG(INFO) << "Creating ORB vocabulary...";
  VocabularyCreator creator;
  creator.create(voc, descriptors, "ORB", params);
  LOG(INFO) << "Vocabulary size: " << voc.size();
}

}
