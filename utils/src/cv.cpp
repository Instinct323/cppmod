#include "utils/cv.hpp"

namespace cv {


cv::Mat kps_grid_mask(const std::vector<KeyPoint> &queryKps, const std::vector<Point2f> &trainKps,
                      const Size &imgSize, const Size &gridSize) {
  int rows = MAX(1, imgSize.height / gridSize.height), cols = MAX(1, imgSize.width / gridSize.width);
  ASSERT(rows * gridSize.height == imgSize.height && cols * gridSize.width == imgSize.width,
         "Invalid grid size " << gridSize << " for image size " << imgSize)
  cv::Mat_<uchar> gridMask(rows * cols, trainKps.size(), uchar(0)), matchMask(queryKps.size(), trainKps.size());
  for (int j = 0; j < trainKps.size(); j++) {
    int r = trainKps[j].y / gridSize.height, c = trainKps[j].x / gridSize.width;
    int rMin = MAX(0, r - 1), rMax = MIN(rows - 1, r + 1);
    int cMin = MAX(0, c - 1), cMax = MIN(cols - 1, c + 1);
    for (int i = rMin; i <= rMax; i++) {
      for (int k = cMin; k <= cMax; k++) {
        gridMask(i * cols + k, j) = 1;
      }
    }
  }
  for (int i = 0; i < queryKps.size(); i++) {
    int r = queryKps[i].pt.y / gridSize.height, c = queryKps[i].pt.x / gridSize.width;
    gridMask.row(r * cols + c).copyTo(matchMask.row(i));
  }
  return matchMask;
}

}
