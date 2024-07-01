#ifndef UTILS__FBOW_HPP
#define UTILS__FBOW_HPP

#include <fbow/fbow.h>
#include <fbow/vocabulary_creator.h>

#include "cv.hpp"
#include "orb.hpp"

namespace fbow {

typedef VocabularyCreator::Params Params;
typedef std::shared_ptr<fbow::Vocabulary> VocabularyPtr;

/**
 * @brief 创建 ORB 特征词汇表
 * @param voc - 词汇表
 * @param extractor - ORB 特征提取器
 * @param loader - 灰度图像加载器
 * @param filenames - 文件名列表
 * @param params - 词汇表创建参数
 */
void createORBvocabulary(Vocabulary &voc,
                         ORB::Extractor *extractor,
                         cv::GrayLoader &loader,
                         std::vector<std::string> &filenames,
                         Params params = Params());

}

#endif
