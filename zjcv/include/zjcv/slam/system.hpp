#ifndef ZJCV__SLAM__SYSTEM_HPP
#define ZJCV__SLAM__SYSTEM_HPP

#include <atomic>
#include <map>
#include <memory>

#include "utils/parallel.hpp"
#include "utils/math.hpp"

#include "frame.hpp"
#include "mappoint.hpp"
#include "tracker.hpp"
#include "viewer.hpp"

namespace slam {

class Frame;

class Mappoint;

class Tracker;

class Viewer;


// require definition of slam::Frame, slam::Tracker, slam::Viewer
#define ZJCV_SLAM_SYSTEM_IMPL \
    ZJCV_SLAM_FRAME_IMPL \
    ZJCV_SLAM_MAPPOINT_IMPL \
    ZJCV_SLAM_TRACKER_IMPL \
    ZJCV_SLAM_VIEWER_IMPL \
    \
    slam::System::System(const YAML::Node &cfg) : mpTracker(new slam::Tracker(this, cfg)), mpViewer(new slam::Viewer(this, cfg)) {} \
    \
    void slam::System::run() { \
      mbRunning = true; \
      mThreads["track"] = parallel::thread_pool.emplace(0, &slam::Tracker::run, mpTracker); \
      mThreads["view"] = parallel::thread_pool.emplace(0, &slam::Viewer::run, mpViewer); \
    }


class System {

public:
    // Subsystems
    std::shared_ptr<Tracker> mpTracker;
    std::shared_ptr<Viewer> mpViewer;
    std::map<std::string, parallel::PriorityThread> mThreads;

    // Status
    std::atomic_bool mbRunning = false;
    std::map<std::string, std::string> mDescs;

    explicit System(const YAML::Node &cfg);

    System(const System &) = delete;

    // Daemons
    void run();

    void stop() {
      mbRunning = false;
      parallel::thread_pool.join();
    }

    // Description
    void set_desc(const std::string &key, const std::string &desc) { mDescs[key] = desc; }

    std::string get_desc() {
      std::string desc;
      for (auto &kv: mDescs) desc += kv.first + "=" + kv.second + ", ";
      desc.erase(desc.end() - 2, desc.end());
      return desc;
    }
};

}

#endif
