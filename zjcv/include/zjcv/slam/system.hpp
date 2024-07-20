#ifndef ZJCV__SLAM__SYSTEM_HPP
#define ZJCV__SLAM__SYSTEM_HPP

#include <atomic>
#include <map>
#include <memory>

#include "utils/parallel.hpp"

#include "atlas.hpp"
#include "frame.hpp"
#include "map.hpp"
#include "mappoint.hpp"
#include "tracker.hpp"
#include "viewer.hpp"

namespace slam {


class System {

public:
    // Subsystems
    ZJCV_BUILTIN std::shared_ptr<Tracker> mpTracker;
    ZJCV_BUILTIN std::shared_ptr<Viewer> mpViewer;
    ZJCV_BUILTIN std::shared_ptr<Atlas> mpAtlas;
    ZJCV_BUILTIN std::map<std::string, parallel::PriorityThread> mThreads;

    // Status
    ZJCV_BUILTIN YAML::Node mCfg;
    ZJCV_BUILTIN std::atomic_bool mbRunning = false;
    ZJCV_BUILTIN std::map<std::string, std::string> mDescs;

    ZJCV_BUILTIN explicit System(const YAML::Node &cfg);

    ZJCV_BUILTIN System(const System &) = delete;

    // Daemons
    ZJCV_BUILTIN void run();

    ZJCV_BUILTIN void stop();

    // Description
    ZJCV_BUILTIN void set_desc(const std::string &key, const std::string &desc) { mDescs[key] = desc; }

    ZJCV_BUILTIN std::string get_desc() {
      std::string desc;
      for (auto &kv: mDescs) desc += kv.first + "=" + kv.second + ", ";
      desc.erase(desc.end() - 2, desc.end());
      return desc;
    }
};

}

#endif
