#pragma once

#include "utils.h"


enum class TrackStatus {
    LOST, INIT, TRACK_BAD, TRACK_GOOD
};


class Frontend {

public:
    typedef cv::GFTTDetector FeatDetector;

    static int nfeats_max, nfeats_init, nfeats_track_bad, nfeats_track_good;

    cv::Ptr<FeatDetector> detector;
    TrackStatus status = TrackStatus::LOST;
    // Frame::Ptr cur_frame = nullptr, last_frame = nullptr;

    Frontend() {
      detector = FeatDetector::create(nfeats_max, 0.01, 20);
    }

    /** @brief 根据特征数量更新状态 */
    void update_status(int nfeats) {
      if (status == TrackStatus::LOST) {
        if (nfeats >= nfeats_init) status = TrackStatus::INIT;
      } else {
        if (nfeats >= nfeats_track_good) {
          status = TrackStatus::TRACK_GOOD;
        } else if (nfeats >= nfeats_track_bad) {
          status = TrackStatus::TRACK_BAD;
        } else { status = TrackStatus::LOST; }
      }
    }
};
