#include "camera.hpp"
#include "utils/math.hpp"
#include "utils/parallel.hpp"

namespace camera {


Base::Ptr from_yaml(const YAML::Node &node) {
  Base::Ptr pCam;
  if (!node.IsNull()) {

    auto type = node["type"].as<std::string>();
    auto imgSize = YAML::toVec<int>(node["resolution"]);
    auto intrinsics = YAML::toVec<float>(node["intrinsics"]);
    auto distCoeffs = YAML::toVec<float>(node["dist_coeffs"]);
    auto T_cam_imu = YAML::toSE3d(node["T_cam_imu"]);

    if (type == "Pinhole") {
      pCam = static_cast<Base::Ptr>(
          new Pinhole({imgSize[0], imgSize[1]}, intrinsics, distCoeffs, T_cam_imu)
      );
    } else if (type == "KannalaBrandt") {
      pCam = static_cast<Base::Ptr>(
          new KannalaBrandt({imgSize[0], imgSize[1]}, intrinsics, distCoeffs, T_cam_imu)
      );
    } else {
      LOG(FATAL) << "Invalid camera type: " << type;
    }
  }
  return pCam;
}


void calib_by_chessboard(std::vector<std::string> &filenames, cv::Mat &distCoeffs,
                         cv::Size boardSize, cv::Size imgSize, int delay) {
  std::vector<std::vector<cv::Point3f>> objPoints;
  std::vector<std::vector<cv::Point2f>> imgPoints;
  // 棋盘角点 3D 坐标
  std::vector<cv::Point3f> objp;
  for (int i = 0; i < boardSize.height; ++i) {
    for (int j = 0; j < boardSize.width; ++j) objp.emplace_back(j, i, 0);
  }
  for (int i = 0; i < filenames.size(); ++i) {
    cv::Mat img = cv::imread(filenames[i]);
    if (!imgSize.empty()) cv::resize(img, img, imgSize);
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    // 找到棋盘角点, 亚像素精确化
    std::vector<cv::Point2f> corners;
    if (!cv::findChessboardCorners(gray, boardSize, corners)) {
      LOG(WARNING) << "Chessboard not found in image[" << i << "] " << filenames[i];
      break;
    }
    cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.01));
    // 保存角点
    objPoints.push_back(objp);
    imgPoints.push_back(corners);
    // 绘制角点
    if (delay > 0) {
      cv::drawChessboardCorners(img, boardSize, corners, true);
      cv::imshow("Chessboard", img);
      cv::waitKey(delay);
    }
  }
  // 读取图像尺寸
  if (imgSize.empty()) imgSize = cv::imread(filenames[0]).size();
  cv::Mat cameraMatrix, rvecs, tvecs;
  cv::calibrateCamera(objPoints, imgPoints, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs);
}


void Base::draw_normalized_plane(const cv::Mat &src, cv::Mat &dst) {
  undistort(src, dst);
  cv::Mat npMap1 = cv::Mat(mImgSize, CV_32FC1), npMap2 = npMap1.clone();
  // 获取归一化平面边界 (桶形畸变)
  float x, y, w, h, W = mImgSize.width - 1, H = mImgSize.height - 1;
  x = this->unproject({0, H / 2}).x, y = this->unproject({W / 2, 0}).y,
  w = this->unproject({W, H / 2}).x - x, h = this->unproject({W / 2, H}).y - y;
  LOG(INFO) << "Normalized plane: " << cv::Vec4f(x, y, x + w, y + h);
  // 计算畸变矫正映射
  for (int r = 0; r < H; ++r) {
    for (int c = 0; c < W; ++c) {
      cv::Point2f p2D = this->project(cv::Point3f(w * c / W + x, h * r / H + y, 1));
      npMap1.at<float>(r, c) = p2D.x;
      npMap2.at<float>(r, c) = p2D.y;
    }
  }
  cv::remap(dst, dst, npMap1, npMap2, cv::INTER_LINEAR);
}


float KannalaBrandt::computeR(float theta) const {
  float theta2 = theta * theta;
  return theta + theta2 * (mvParam[4] + theta2 * (mvParam[5] + theta2 * (mvParam[6] + theta2 * mvParam[7])));
}


void KannalaBrandt::make_unproject_cache() {
  float wx, wy, wz;
  for (int r = 0; r < mImgSize.height; ++r) {
    wy = (r - mvParam[3]) / mvParam[1];
    for (int c = 0; c < mImgSize.width; ++c) {
      wx = (c - mvParam[2]) / mvParam[0];
      wz = this->solveWZ(wx, wy);
      mUnprojectCache.at<cv::Vec2f>(r, c) = {wx / wz, wy / wz};
    }
  }
}


float KannalaBrandt::solveWZ(float wx, float wy, size_t iterations) const {
  // wz = lim_{theta -> 0} R / tan(theta) = 1
  float wz = 1.f;
  float R = hypot(wx, wy);
  if (R > KANNALA_BRANDT_UNPROJECT_PRECISION) {
    float theta = KANNALA_BRANDT_MAX_FOV;
    if (R < this->computeR(theta)) {
      // 最小化损失: (poly(theta) - R)^2
      int i = 0;
      float e;
      for (; i < iterations; i++) {
        float theta2 = theta * theta, theta4 = theta2 * theta2, theta6 = theta4 * theta2, theta8 = theta6 * theta2;
        float k0_theta2 = mvParam[4] * theta2, k1_theta4 = mvParam[5] * theta4,
            k2_theta6 = mvParam[6] * theta6, k3_theta8 = mvParam[7] * theta8;
        e = theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - R;
        if (abs(e) < R * KANNALA_BRANDT_UNPROJECT_PRECISION) break;
        // 梯度下降法: g = (poly(theta) - R) / poly'(theta)
        theta -= e / (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 + 9 * k3_theta8);
      }
      if (i == iterations) LOG(WARNING) << "solveWZ(" << wx << ", " << wy << "): relative error " << abs(e) / R;
    }
    wz = R / tanf(theta);
  }
  return wz;
}


void KannalaBrandt::stereoORBfeatures(Base *pCamRight,
                                      ORB::Extractor *pExtractor0, ORB::Extractor *pExtractor1,
                                      const cv::Mat &img0, const cv::Mat &img1,
                                      ORB::KeyPoints &kps0, ORB::KeyPoints &kps1,
                                      cv::Mat &desc0, cv::Mat &desc1, std::vector<cv::DMatch> &matches) {
  ASSERT(pCamRight->get_type() == CameraType::KANNALA_BRANDT, "Camera type must be KannalaBrandt")
  // 特征提取
  int lapCnt0, lapCnt1;
  STEREO_ORB_EXTRACT(lapCnt0, lapCnt1)
  // 双目匹配
  auto matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
  // Lowe's ratio test
  std::vector<std::vector<cv::DMatch>> knn_matches;
  matcher->knnMatch(desc0.rowRange(0, lapCnt0), desc1.rowRange(0, lapCnt1), knn_matches, 2);
#define LOWE_S_RADIO 0.7
  for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it) {
    if (it->size() < 2) continue;
    cv::DMatch &m0 = (*it)[0], &m1 = (*it)[1];
    if (m0.distance < LOWE_S_RADIO * m1.distance) matches.push_back(m0);
  }
}


void Pinhole::stereo_rectify(Pinhole *cam_right) {
  ASSERT(this->mImgSize == cam_right->mImgSize, "Image size must be the same")
  Sophus::SE3d Trl = this->T_cam_imu.inverse() * cam_right->T_cam_imu;
  cv::Mat P1, P2, Q;
  // 双目矫正
  cv::stereoRectify(this->getK(), this->get_distcoeffs(), cam_right->getK(), cam_right->get_distcoeffs(), mImgSize,
                    Eigen::toCvMat<double>(Trl.rotationMatrix()), Eigen::toCvMat<double>(Trl.translation()),
                    this->mRectR, cam_right->mRectR, P1, P2, Q);
  // 重新初始化畸变矫正映射
  cv::initUndistortRectifyMap(this->getK(), this->get_distcoeffs(), this->mRectR, P1, mImgSize, CV_32FC1, this->mMap1, this->mMap2);
  cv::initUndistortRectifyMap(cam_right->getK(), cam_right->get_distcoeffs(), cam_right->mRectR, P2, mImgSize, CV_32FC1,
                              cam_right->mMap1, cam_right->mMap2);
  // 原地修改相机内参
  int paramPos[2][4] = {{0, 1, 0, 1},
                        {0, 1, 2, 2}};
  for (int i = 0; i < 4; i++) {
    this->set_param(i, P1.at<double>(paramPos[0][i], paramPos[1][i]), true);
    cam_right->set_param(i, P2.at<double>(paramPos[0][i], paramPos[1][i]), true);
  }
  // 原地修改相机位姿
  Sophus::SE3d R1(cv::toEigen<double>(this->mRectR), Eigen::Vector3d::Zero()),
      R2(cv::toEigen<double>(cam_right->mRectR), Eigen::Vector3d::Zero());
  this->T_cam_imu = R1 * this->T_cam_imu;
  cam_right->T_cam_imu = R2 * cam_right->T_cam_imu;
}


void Pinhole::stereoORBfeatures(Base *pCamRight,
                                ORB::Extractor *pExtractor0, ORB::Extractor *pExtractor1,
                                const cv::Mat &img0, const cv::Mat &img1,
                                ORB::KeyPoints &kps0, ORB::KeyPoints &kps1,
                                cv::Mat &desc0, cv::Mat &desc1, std::vector<cv::DMatch> &matches) {
  ASSERT(pCamRight->get_type() == CameraType::PINHOLE, "Camera type must be Pinhole")
  // 特征提取
  int lapCnt0, lapCnt1;
  STEREO_ORB_EXTRACT(lapCnt0, lapCnt1)
  // 分配右相机特征到行
  cv::Mat_<uchar> rowMask(img0.rows, lapCnt1, uchar(0)), matchMask(lapCnt0, lapCnt1, uchar(0));
  for (int j = 0; j < lapCnt1; ++j) {
    float y = kps1[j].pt.y, r = kps1[j].size / 2;
    int t = MAX(0, cvFloor(y - r)), b = MIN(img0.rows - 1, cvCeil(y + r));
    for (int i = t; i <= b; ++i) rowMask.at<uchar>(i, j) = uchar(1);
  }
  for (int i = 0; i < lapCnt0; ++i) {
    int y = cvRound(kps0[i].pt.y);
    rowMask.row(y).copyTo(matchMask.row(i));
  }
  // 双目匹配
  auto matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
  std::vector<cv::DMatch> tmp_matches;
  matcher->match(desc0.rowRange(0, lapCnt0), desc1.rowRange(0, lapCnt1), tmp_matches, matchMask);
  // Pauta Criterion
  std::vector<float> dx;
  for (auto &m: tmp_matches) dx.push_back(kps0[m.queryIdx].pt.x - kps1[m.trainIdx].pt.x);
  math::PautaCriterion<float> is_inlier(dx, 1);
  for (int i = 0; i < tmp_matches.size(); ++i) {
    if (!is_inlier(dx[i])) continue;
    matches.push_back(tmp_matches[i]);
  }
  matches.shrink_to_fit();
  // todo: Subpixel refinement
}

}
