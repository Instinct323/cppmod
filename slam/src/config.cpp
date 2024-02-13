#include <opencv2/features2d.hpp>
#include "frame.h"
#include "mappoint.h"

int Frame::nfeats_max = 200;
int Frame::nfeats_min = 20;
float Frame::nfeats_decay = 0.8;

cv::Ptr<cv::GFTTDetector> Frame::detector = cv::GFTTDetector::create(Frame::nfeats_max, 0.01, 20);

double Mappoint::z_floor = - 2048.;
