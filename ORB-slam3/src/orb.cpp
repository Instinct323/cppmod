#include "orb.hpp"

namespace ORB {

bool search_by_project(slam::feature::Frame *pRef, slam::feature::Frame *pCur, std::vector<cv::DMatch> &matches) {
  slam::Tracker::Ptr &pTracker = pCur->mpSystem->mpTracker;
  const camera::Base::Ptr &pCam0 = pTracker->mpCam0;

  // 利用已有地图点进行匹配
  cv::GridDict grid_dict(pCur->mvKps0.begin(), pCur->mvKps0.end(), pCur->mImg0.size(), {pTracker->GRID_SIZE, pTracker->GRID_SIZE});
  Sophus::SE3f T_world_cam0 = pCam0->T_cam_imu.inverse() * pCur->mPose.T_world_imu;
  for (int d = 0; d <= 2; d++) {
    cv::Mat_<uchar> mask(pRef->mvKps0.size(), pCur->mvKps0.size(), uchar(0));

    // 将地图点投影到当前帧
    for (auto &pair: pRef->mmpMappts) {
      if (pair.second->is_invalid()) continue;
      pair.second->prune();
      cv::Point2f pt = pCam0->project(T_world_cam0 * pair.second->mPos);
      if (!pCam0->is_in(pt)) continue;
      grid_dict(pt.x, pt.y, d).copyTo(mask.row(pair.first));
    }

    matches.clear();
    pTracker->mpMatcher->search(pRef->mDesc0, pCur->mDesc0, mask, matches);
    if (matches.empty()) LOG(WARNING) << "No matches found";
    cv::chi2_filter(matches, 5.991);
    cv::make_one2one(matches, true);

    if (matches.size() > pTracker->MIN_MATCHES) return true;
  }
  return false;
}

}
