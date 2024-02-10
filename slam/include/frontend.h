#pragma once

#include "utils.h"


class Frontend {

public:
    typedef cv::GFTTDetector FeatDetector;

    cv::Ptr<FeatDetector> detector;
    // Frame::Ptr cur_frame = nullptr, last_frame = nullptr;

    Frontend() {
      detector = FeatDetector::create(nfeats_max, 0.01, 20);
    }
};
