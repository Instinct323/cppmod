#pragma once

#include "camera.h"
#include "keypoint.h"


class Frame {

public:
    typedef cv::GFTTDetector FeatDetector;
    typedef std::shared_ptr<Frame> Ptr;

    cv::Mat img;
    std::vector<Keypoint> kps;    // 关键点
    std::vector<uchar> status;   // 跟踪状态
    SE3 Tlc;   // 相机运动 (lastWorld <- curWorld)

    explicit Frame(cv::Mat img,
                   const Ptr &last_frame,
                   Camera::Ptr camera,
                   uint id,
                   uint id_kframe = 0,
                   cv::Ptr<cv::Feature2D> detector = nullptr
    ) : img(img), Tlc(last_frame->Tlc) {

      std::vector<cv::Point2f> last_kps, cur_kps;

      // 暂时修改相机位姿
      SE3 Trl = camera->Trw;  // robot <- lastWorld
      camera->set_Trw(Trl * Tlc);
      for (int i = 0; i < last_frame->kps.size(); i++) {
        Keypoint kp = last_frame->kps[i];
        last_kps.push_back(kp);
        // 使用路标初始化 kps
        auto mp = kp.getMappoint();
        if (mp != nullptr) kp = camera->world2pixel(mp->pos);
        cur_kps.push_back(kp);
      }
      // 还原相机位姿
      camera->set_Trw(Trl);

      if (match_keypoints(last_frame, last_kps, cur_kps) < Frontend::nfeats_track_good && detector != nullptr) {
        // 匹配到的特征点不足, 检测新特征点
        std::vector<cv::KeyPoint> org_kps;
        detector->detect(img, org_kps);
        for (int i = 0; i < org_kps.size(); i++) { kps.push_back(org_kps[i].pt); }
        // todo: 创建新的路标点
      } else {
        // 转化存储
        for (int i = 0; i < last_kps.size(); i++) {
          kps.push_back(Keypoint(cur_kps[i], last_frame->kps[i]._mappoint));
        }
        reduce();
      }
    }

    /** @brief LK 光流匹配关键点 */
    int match_keypoints(const Ptr &last_frame,
                        std::vector<cv::Point2f> &last_kps,
                        std::vector<cv::Point2f> &cur_kps) {
      cv::calcOpticalFlowPyrLK(
          last_frame->img, img, last_kps, cur_kps,
          status, cv::Mat(), cv::Size(11, 11), 3,
          cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
          cv::OPTFLOW_USE_INITIAL_FLOW);
      return std::count(status.begin(), status.end(), 1);
    }

    /** @brief 删除匹配失败的关键点 (仅运行一次) */
    void reduce() {
      // 删除匹配失败的关键点, 过期的路标点
      for (int i = status.size() - 1; i >= 0; i--) {
        if (status[i] == 0 || kps[i].hasMappoint()) kps.erase(kps.begin() + i);
      }
      // 更新路标点的关键点
      for (int i = 0; i < kps.size(); i++) {
        auto mp = kps[i].getMappoint();
        if (mp != nullptr) mp->add(Ptr(this), i);
      }
      // 清空状态
      status.clear();
      kps.shrink_to_fit();
    }
};
