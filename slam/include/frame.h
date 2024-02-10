#pragma once

#include "camera.h"
#include "keypoint.h"


enum class TrackStatus {
    LOST, BAD, GOOD
};


/** @brief 帧, 匹配的特征点数量取决于给定帧的特征点 (与路标点无关)
 * 匹配成功的点数量不足时, 将使用特征提取器, 路标点于下一帧中补充 */
class Frame {

public:
    typedef cv::Ptr<cv::Feature2D> DetectorPtr;
    typedef std::shared_ptr<Frame> Ptr;

    static int nfeats_max, nfeats_bad, nfeats_good;
    Ptr shared_this;

    Camera::Ptr camera;
    cv::Mat img;
    std::vector<Keypoint> kps;    // 关键点
    std::vector<uchar> kp_status;   // 跟踪状态

    TrackStatus status = TrackStatus::GOOD;
    bool is_init = false, has_Tcw = false;
    SE3 _Tcw;

    // 构造方法
    static Ptr create(const Camera::Ptr &camera,
                      const cv::Mat &img,
                      const Ptr &last_frame,
                      const SE3 &T01,
                      const DetectorPtr &detector = nullptr) {
      Ptr p = Ptr(new Frame(camera, img, last_frame, T01, detector));
      p->shared_this = p;
      return p;
    }

    explicit Frame(const Camera::Ptr &camera,
                   const cv::Mat &img,
                   const Ptr &last_frame,
                   const SE3 &T01,   // 相机运动 (lastWorld <- curWorld)
                   const DetectorPtr &detector = nullptr);

    /** @brief LK 光流匹配关键点 */
    int match_keypoints(const Ptr &last_frame,
                        std::vector<cv::Point2f> &last_kps,
                        std::vector<cv::Point2f> &cur_kps) {
      cv::calcOpticalFlowPyrLK(
          last_frame->img, img, last_kps, cur_kps,
          kp_status, cv::Mat(), cv::Size(11, 11), 3,
          cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
          cv::OPTFLOW_USE_INITIAL_FLOW);
      return std::count(kp_status.begin(), kp_status.end(), 1);
    }

    bool is_keyframe() { return is_init && status != TrackStatus::LOST; }

    void set_pose(const SE3 *pose = nullptr);

    bool get_pose(std::vector<SE3> &T) const {
      if (has_Tcw) { T.push_back(_Tcw); } else { LOG(WARNING) << "Frame: Tcw is not set!"; }
      return has_Tcw;
    }

    /** @brief 更新状态, 必要时提取新的特征点 */
    void switch_status(int nfeats,
                       const Ptr &last_frame,
                       const DetectorPtr &detector) {
      if (nfeats >= nfeats_good) {
        status = TrackStatus::GOOD;
      } else {
        is_init = detector != nullptr && !last_frame->is_init;
        if (is_init) {
          kp_status.clear();
          // 匹配到的特征点不足 (上一帧不是刚初始化的), 检测新特征点
          std::vector<cv::KeyPoint> org_kps;
          detector->detect(img, org_kps);
          for (auto &org_kp: org_kps) { kps.emplace_back(org_kp.pt); }
          nfeats = kps.size();
        }
        status = (nfeats >= nfeats_bad) ? TrackStatus::BAD : TrackStatus::LOST;
      }
    }

    /** @brief 存储整理 (仅在构造方法中运行) */
    void reduce(const Ptr &last_frame,
                std::vector<cv::Point2f> &cur_kps) {
      for (int i = 0; i < kp_status.size(); i++) {
        // 转化存储: cur_kps -> kps (删除匹配失败的关键点, 过期的路标点)
        if (kp_status[i] != 0 && last_frame->kps[i].hasMappoint()) {
          Mappoint::Ptr mp;
          // 上一帧是初始化状态, 创建路标点
          if (last_frame->is_init) {
            mp = Mappoint::create();
            mp->add(last_frame, i);
          } else {
            mp = last_frame->kps[i].getMappoint();
          }
          // 更新路标点的关键点
          if (mp != nullptr && mp->is_inlier) {
            mp->add(shared_this, kps.size());
            mp->triangulation();
            kps.emplace_back(cur_kps[i], mp);
          }
        }
      }
      // 清空状态
      kp_status.clear();
      kps.shrink_to_fit();
    }
};
