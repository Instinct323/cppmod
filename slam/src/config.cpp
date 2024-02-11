#include <opencv2/features2d.hpp>
#include "frame.h"
#include "mappoint.h"

int Frame::nfeats_max = 200;
int Frame::nfeats_bad = 20;
int Frame::nfeats_good = 50;

cv::Ptr<cv::GFTTDetector> Frame::detector = cv::GFTTDetector::create(nfeats_max, 0.01, 20);

double Mappoint::z_floor = 0.;
