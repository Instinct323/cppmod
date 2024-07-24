#include "utils/cv.hpp"

namespace cv {

GridMask::GridMask(const std::vector<KeyPoint> &trainKps, const Size &imgSize,
                   const Size &gridSize, int dilation
) : mRows(std::max(1, imgSize.height / gridSize.height)), mCols(std::max(1, imgSize.width / gridSize.width)),
    mGridSize(gridSize), mMask(mRows * mCols, trainKps.size(), uchar(0)) {
  assert(dilation >= 0);

  for (int j = 0; j < trainKps.size(); j++) {
    float radius = trainKps[j].size / 2;
    int rMin = std::max(0, int((trainKps[j].pt.y - radius) / mGridSize.height - dilation)),
        rMax = std::min(mRows, int((trainKps[j].pt.y + radius) / mGridSize.height + dilation)),
        cMin = std::max(0, int((trainKps[j].pt.x - radius) / mGridSize.width - dilation)),
        cMax = std::min(mCols - 1, int((trainKps[j].pt.x + radius) / mGridSize.width + dilation));

    for (int r = rMin; r <= rMax; r++) {
      for (int c = cMin; c <= cMax; c++) {
        mMask(r * mCols + c, j) = 1;
      }
    }
  }
}

}
