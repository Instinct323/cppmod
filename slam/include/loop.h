#pragma once

#include "frame.h"
#include "utils.h"


class Monocular {

public:
    cv::VideoCapture video;
    Camera::Ptr camera;

    Frame::Ptr cur_frame = nullptr, last_frame = nullptr;
    SE3 Tcw, motion;

    std::vector<Frame::Ptr> keyframes;

    // 加载视频文件
    explicit Monocular(const std::string& file) : video(file) {
      if (!video.isOpened()) {
        LOG(FATAL) << "Failed to open video file: " << file;
      }
    }

    void frontend_loop() {
      while (true) {
        last_frame = cur_frame;
        cv::Mat img;
        video >> img;
        cur_frame = Frame::create(img, camera, last_frame);
        // cur: TRACK_BAD 关键帧, 保存并重新初始化
        if (cur_frame->status == TRACK_BAD) {
          keyframes.push_back(cur_frame);
          cur_frame = Frame::create(img, camera, nullptr);
        }
        // cur: INIT_FAILED / TRACK_LOST 迷失
        if (cur_frame->status == INIT_FAILED || cur_frame->status == TRACK_LOST) {
          LOG(ERROR) << "Trace failed.";
          break;
        }
        // 求解位姿信息
        cur_frame->mul_Tcw(motion);
        motion = cur_frame->get_Tcw() * Tcw.inverse();
        Tcw = cur_frame->get_Tcw();
      }
    }
};
