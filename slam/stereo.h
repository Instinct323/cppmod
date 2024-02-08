#include "rcv.h"


class Frame : public ImgPair {

public:
    typedef std::shared_ptr<Frame> Ptr;

    int id, id_keyframe;
    SE3 Tcw;
    std::vector<cv::Point2f> feat1, feat2;

};
