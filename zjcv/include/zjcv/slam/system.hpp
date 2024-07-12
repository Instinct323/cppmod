#ifndef ZJCV__SLAM__SYSTEM_HPP
#define ZJCV__SLAM__SYSTEM_HPP

#include <atomic>
#include <map>
#include <memory>

#include "frame.hpp"
#include "tracker.hpp"
#include "utils/parallel.hpp"
#include "utils/math.hpp"
#include "viewer.hpp"

namespace slam {


template<
    template<typename> typename TrackerTemplate,
    template<typename> typename FrameTemplate,
    template<typename> typename ViewerTemplate,
    typename Storage = bool>
class System {

public:
    typedef TrackerTemplate<System> Tracker;
    typedef FrameTemplate<System> Frame;
    typedef ViewerTemplate<System> Viewer;

    // Type check
    static_assert(std::is_base_of<TrackerBase<System>, Tracker>::value, "TrackerTemplate must be derived from TrackerBase<System>");
    static_assert(std::is_base_of<FrameBase<System>, Frame>::value, "FrameTemplate must be derived from FrameBase<System>");
    static_assert(std::is_base_of<ViewerBase<System>, Viewer>::value, "ViewerTemplate must be derived from ViewerBase<System>");

    // Subsystems
    const typename std::shared_ptr<Tracker> mpTracker;
    const typename std::shared_ptr<Viewer> mpViewer;
    std::map<std::string, parallel::PriorityThread> mThreads;
    Storage mStorage;

    // Status
    std::atomic_bool mbRunning = false;

    explicit System(const YAML::Node &cfg
    ) : mpTracker(new Tracker(this, cfg)), mpViewer(new Viewer(this, cfg)) {};

    System(const System &) = delete;

    // Daemons
    void run() {
      mbRunning = true;
      mThreads["track"] = parallel::thread_pool.emplace(0, &Tracker::run, mpTracker);
      mThreads["view"] = parallel::thread_pool.emplace(0, &Viewer::run, mpViewer);
    }

    void stop() {
      mbRunning = false;
      parallel::thread_pool.join();
    }
};

}

#endif
