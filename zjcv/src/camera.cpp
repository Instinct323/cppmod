#include "utils/cv.hpp"
#include "utils/file.hpp"
#include "zjcv/camera.hpp"

namespace camera {


Base::Ptr from_yaml(const YAML::Node &cfg) {
  Base::Ptr pCam;
  if (!YAML::is_invalid(cfg)) {

    auto type = cfg["type"].as<std::string>();
    auto imgSize = YAML::toVec<int>(cfg["resolution"]);
    auto intrinsics = YAML::toVec<float>(cfg["intrinsics"]);
    auto distCoeffs = YAML::toVec<float>(cfg["dist_coeffs"]);
    auto T_cam_imu = YAML::toSE3<float>(cfg["T_cam_imu"]);

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


void Base::draw_normalized_plane(const cv::Mat &src, cv::Mat &dst) const {
  undistort(src, dst);
  cv::Mat npMap1 = cv::Mat(img_size, CV_32FC1), npMap2 = npMap1.clone();
  // 获取归一化平面边界 (桶形畸变)
  float x, y, w, h, W = img_size.width - 1, H = img_size.height - 1;
  x = this->unproject({0, H / 2})[0], y = this->unproject({W / 2, 0})[1],
  w = this->unproject({W, H / 2})[0] - x, h = this->unproject({W / 2, H})[1] - y;
  LOG(INFO) << "Normalized plane: " << cv::Vec4f(x, y, x + w, y + h);
  // 计算畸变矫正映射
  for (int r = 0; r < H; ++r) {
    for (int c = 0; c < W; ++c) {
      cv::Point2f p2D = this->project(Eigen::Vector3f(w * c / W + x, h * r / H + y, 1));
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
  for (int r = 0; r < img_size.height; ++r) {
    wy = (r - mvParam[3]) / mvParam[1];
    for (int c = 0; c < img_size.width; ++c) {
      wx = (c - mvParam[2]) / mvParam[0];
      wz = this->solveWZ(wx, wy);
      mUnprojectCache.at<cv::Vec3f>(r, c) = {wx / wz, wy / wz, 1};
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

}
