#pragma once

#include <chrono>
#include <Eigen/Core>
#include <glog/logging.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

typedef Sophus::SE3d SE3;
typedef Eigen::Matrix3d Mat33;
typedef Eigen::Vector3d Vec3;

typedef Eigen::Matrix2d Mat22;
typedef Eigen::Vector2d Vec2;


/** @brief 日志 */
class Logger {
public:
    explicit Logger(char **argv) {
      google::InitGoogleLogging(argv[0]);
      FLAGS_logtostderr = true;
      FLAGS_minloglevel = google::INFO;
    }

    ~Logger() { google::ShutdownGoogleLogging(); }
};


/** @brief 计时器 */
class Timer {

public:
    typedef std::chrono::steady_clock Clock;
    typedef Clock::time_point Timepoint;
    typedef std::chrono::duration<double> Duration;

    Timepoint t0;

    Timer() { t0 = Clock::now(); }

protected:
    friend std::ostream &operator<<(std::ostream &os, const Timer &timer) {
      Timepoint t1 = Clock::now();
      Duration time_used = std::chrono::duration_cast<Duration>(t1 - timer.t0);
      return (os << time_used.count());
    }
};
