#include "rcv.h"

using namespace std;

class Frame : public ImgPair {

public:
    int id, id_keyframe;
    SE3 Tcw;
    vector<cv::Point2f> feat1, feat2;

};
