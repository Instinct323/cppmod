#ifndef ORBSLAM__FRAME_HPP
#define ORBSLAM__FRAME_HPP

#include "utils/orb.hpp"
#include "zjcv/slam/frame.hpp"

namespace slam {

template<typename System>
class Frame : public FrameBase<System> {

public:
    using FrameBase<System>::FrameBase;

    // Features
    ORB::KeyPoints mvKps0, mvKps1;
    cv::Mat mDesc0, mDesc1;
    std::vector<cv::DMatch> mStereoMatches;

    virtual void process() override {
      auto &pTracker = this->mpSystem->mpTracker;
      // 特征点提取、去畸
      if (pTracker->is_monocular()) {
        pTracker->mpCam0->monoORBfeatures(pTracker->mpExtractor0.get(), this->mImg0, mvKps0, mDesc0);
      } else {
        pTracker->mpCam0->stereoORBfeatures(pTracker->mpCam1.get(), pTracker->mpExtractor0.get(), pTracker->mpExtractor1.get(),
                                            this->mImg0, this->mImg1, mvKps0, mvKps1, mDesc0, mDesc1, mStereoMatches);
      }
    }

};

}

#endif
