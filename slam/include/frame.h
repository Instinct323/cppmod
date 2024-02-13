#pragma once

#include "camera.h"
#include "keypoint.h"


enum FrameStatus {
    INIT_FAILED, INIT_SUCCESS,
    TRACK_LOST, TRACK_BAD, TRACK_GOOD
};


/** @brief 帧, 匹配的特征点数量取决于给定帧的特征点 (与路标点无关)
 * 匹配成功的点数量不足时, 将使用特征提取器, 路标点于下一帧中补充 */
class Frame {

public:
    typedef std::shared_ptr<Frame> Ptr;

    static int nfeats_max, nfeats_min;
    static float nfeats_decay;
    static cv::Ptr<cv::GFTTDetector> detector;   // 特征提取器

    std::weak_ptr<Frame> weak_this;
    Ptr link = nullptr;   // 链接所跟踪特征的第一帧

    cv::Mat img;
    Camera::Ptr camera;
    std::vector<Keypoint> kps;    // 关键点
    std::vector<uchar> kp_status;   // 关键点跟踪状态

    FrameStatus status;
    bool has_Tcw = false;
    SE3 Tcw;

    static Ptr create(const cv::Mat &img,
                      const Camera::Ptr &camera,
                      const Ptr &last_frame) {
      Ptr p = Ptr(new Frame(img, camera, last_frame));
      p->weak_this = p;
      return p;
    }

    void mul_Tcw(const SE3 &motion, bool optimize = true, double chi2_th = 5.991);

    bool get_Tcw(std::vector<SE3> &T) const {
      if (has_Tcw) { T.push_back(Tcw); } else { LOG(WARNING) << "Frame: Tcw is not set!"; }
      return has_Tcw;
    }

    bool get_Tcw(SE3 &T) const {
      if (has_Tcw) { T = Tcw; } else { LOG(WARNING) << "Frame: Tcw is not set!"; }
      return has_Tcw;
    }

protected:
    // 追踪上一帧
    Frame(const cv::Mat &img,
          const Camera::Ptr &camera,
          const Ptr &last_frame = nullptr);

    /** @brief LK 光流匹配关键点 */
    void match_keypoints(const Ptr &last_frame,
                         std::vector<cv::Point2f> &last_kps,
                         std::vector<cv::Point2f> &cur_kps) {
      cv::calcOpticalFlowPyrLK(
          last_frame->img, img, last_kps, cur_kps,
          kp_status, cv::Mat(), cv::Size(11, 11), 3,
          cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
          cv::OPTFLOW_USE_INITIAL_FLOW);
    }

    /** @brief 存储整理 (仅在构造方法中运行) */
    int reduce(const Ptr &last_frame,
               std::vector<cv::Point2f> &cur_kps);
};
