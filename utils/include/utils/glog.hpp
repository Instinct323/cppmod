#ifndef ZJCV__LOGGING_HPP
#define ZJCV__LOGGING_HPP

#include <chrono>
#include <glog/logging.h>

#define ASSERT(expr, msg) if (!(expr)) { LOG(FATAL) << "AssertionError: " << msg; }

namespace glog {


// 日志
class Logger {
public:
    explicit Logger(char **argv) {
        google::InitGoogleLogging(argv[0]);
        FLAGS_logtostderr = true;
        FLAGS_minloglevel = google::INFO;
    }

    ~Logger() { google::ShutdownGoogleLogging(); }
};


// 计时器
class Timer {

public:
    typedef std::chrono::system_clock Clock;
    typedef Clock::time_point Timepoint;
    typedef std::chrono::duration<double> Duration;

    Timepoint t0;

    Timer() { reset(); }

    void reset() { t0 = Clock::now(); }

    double elapsed() const {
        Timepoint t1 = Clock::now();
        Duration time_used = std::chrono::duration_cast<Duration>(t1 - t0);
        return time_used.count();
    }
};

}

#endif
