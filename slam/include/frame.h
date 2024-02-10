#pragma once

#include "camera.h"
#include "keypoint.h"


/** @brief 帧, 匹配的特征点数量取决于给定帧的特征点 (与路标点无关)
 * 匹配成功的点数量不足时, 将使用特征提取器, 路标点于下一帧中补充 */
class Frame {

public:
    typedef cv::GFTTDetector FeatDetector;
    typedef std::shared_ptr<Frame> Ptr;

    Camera::Ptr camera;
    cv::Mat img;
    std::vector<Keypoint> kps;    // 关键点
    std::vector<uchar> status;   // 跟踪状态

    bool is_init, has_Tcw = false;
    SE3 _Tcw;

    // 相机位姿 (camera <- world)
    void setTcw(const SE3 &Tcw) {
      _Tcw = Tcw;
      has_Tcw = true;
    }

    bool get_Tcw(std::vector<SE3> &T) const {
      if (has_Tcw) { T.push_back(_Tcw); } else { LOG(WARNING) << "Frame: Tcw is not set!"; }
      return has_Tcw;
    }

    explicit Frame(const Camera::Ptr &camera,
                   const cv::Mat &img,
                   const Ptr &last_frame,
                   const SE3 &T01,   // 相机运动 (lastWorld <- curWorld)
                   const cv::Ptr<cv::Feature2D> &detector = nullptr);

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

    /** @brief 存储整理 (仅运行一次) */
    void reduce() {
      if (!is_init) {
        // 删除匹配失败的关键点, 过期的路标点
        for (int i = status.size() - 1; i >= 0; i--) {
          if (status[i] == 0 || kps[i].hasMappoint()) kps.erase(kps.begin() + i);
        }
      }
      if (!status.empty()) {
        // 更新路标点的关键点
        for (int i = 0; i < kps.size(); i++) {
          auto mp = kps[i].getMappoint();
          if (mp != nullptr) mp->add(Ptr(this), i);
        }
        // 清空状态
        status.clear();
      }
      kps.shrink_to_fit();
    }
};
