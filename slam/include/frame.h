#pragma once

#include "camera.h"
#include "keypoint.h"


enum FrameStatus {
    INIT_FAILED, INIT_SUCCESS,
    TRACK_LOST, TRACK_BAD, TRACK_GOOD
};


/** @brief 帧, 匹配的特征点数量不多于给定帧 (与路标点无关) */
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

    FrameStatus status;
    SE3 Tcw;
    bool has_Tcw = false;

    static Ptr create(const cv::Mat &img,
                      const Camera::Ptr &camera,
                      const Ptr &last_frame) {
      Ptr p = Ptr(new Frame(img, camera, last_frame));
      p->weak_this = p;
      return p;
    }

    void mul_Tcw(const SE3 &motion, bool optimize = true, double chi2_th = 5.991);

    inline SE3 get_Tcw() const {
      if (!has_Tcw) { LOG(WARNING) << *this << ": Tcw is not set!"; }
      return Tcw;
    }

protected:
    // 追踪上一帧
    explicit Frame(const cv::Mat &img,
          const Camera::Ptr &camera,
          const Ptr &last_frame = nullptr);

    /** @brief LK 光流匹配关键点 */
    int match_keypoints(const Ptr &last_frame,
                         std::vector<cv::Point2f> &last_kps,
                         std::vector<cv::Point2f> &cur_kps);

    friend std::ostream &operator<<(std::ostream &os, const Frame &frame) {
      return os << "Frame(" << frame.kps.size() << ")";
    }
};
