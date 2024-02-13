#include <opencv2/features2d.hpp>

#include "frame.h"
#include "mappoint.h"

std::vector<std::weak_ptr<Mappoint>> Mappoint::map;

int Frame::nfeats_min = 20;
float Frame::nfeats_decay = 0.8;

cv::Ptr<cv::GFTTDetector> Frame::detector = cv::GFTTDetector::create(200, 0.01, 20);

double Mappoint::z_floor = -2048.;
