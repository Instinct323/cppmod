#pragma once

#include "frame.h"
#include "utils.h"


class Monocular {

public:
    Frame::Ptr cur_frame = nullptr, last_frame = nullptr;
    SE3 Tcw, motion;

    cv::VideoCapture video;
    Camera::Ptr camera;

    // 加载视频文件
    Monocular(std::string file) : video(file) {
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

        // 当前帧初始化失败 / 追踪失败, 迷失

        // 当前帧初始化成功, 关键帧

        // 当前帧正常追踪, 非关键帧
      }
    }
};


class Stereo {

public:
    // Frame::Ptr cur_frame = nullptr, last_frame = nullptr;
};
