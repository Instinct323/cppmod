#ifndef ZJSLAM__CAMERA__CALIB_HPP
#define ZJSLAM__CAMERA__CALIB_HPP

#include <opencv2/opencv.hpp>

#include "../logging.hpp"

namespace camera {


void calibByChessboard(std::vector<std::string> &filenames, cv::Mat distCoeffs,
                       cv::Size boardSize, cv::Size imgSize = {-1, -1}, int delay = 1) {
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

}

#endif
