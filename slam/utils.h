#pragma once

#include <chrono>
#include <glog/logging.h>
#include <iostream>

using namespace std;


/** @brief 日志类 */
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
    typedef chrono::steady_clock Clock;
    typedef Clock::time_point Timepoint;
    typedef chrono::duration<double> Duration;

    Timepoint t0;

    Timer() { t0 = Clock::now(); }

protected:
    friend ostream &operator<<(ostream &os, const Timer &timer) {
      Timepoint t1 = Clock::now();
      Duration time_used = chrono::duration_cast<Duration>(t1 - timer.t0);
      return (os << time_used.count());
    }
};
