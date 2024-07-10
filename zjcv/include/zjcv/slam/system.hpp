#ifndef ORBSLAM__SYSTEM_HPP
#define ORBSLAM__SYSTEM_HPP

#include <atomic>
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
    template<typename> typename ViewerTemplate
>
class SystemBase {

public:
    typedef TrackerTemplate<SystemBase> Tracker;
    typedef FrameTemplate<Tracker> Frame;
    typedef ViewerTemplate<SystemBase> Viewer;

    // Type check
    static_assert(std::is_base_of<TrackerBase<SystemBase>, Tracker>::value, "TrackerTemplate must be derived from TrackerBase<SystemBase>");
    static_assert(std::is_base_of<FrameBase<Tracker>, Frame>::value, "FrameTemplate must be derived from FrameBase<TrackerBase>");
    static_assert(std::is_base_of<ViewerBase<SystemBase>, Viewer>::value, "ViewerTemplate must be derived from ViewerBase<SystemBase>");

    // Subsystems
    const typename std::shared_ptr<Tracker> mpTracker;
    const typename std::shared_ptr<Viewer> mpViewer;

    // Status
    std::atomic_bool mbRunning = false;

    explicit SystemBase(YAML::Node cfg
    ) : mpTracker(new Tracker(this, cfg)), mpViewer(new Viewer(this)) {};

    SystemBase(const SystemBase &) = delete;

    // Daemons
    void run() {
      mbRunning = true;
      parallel::thread_pool.emplace(0, &Viewer::run, mpViewer);
    }

    void stop() {
      mbRunning = false;
      parallel::thread_pool.join();
    }
};

}

#endif
