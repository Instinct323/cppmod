#ifndef ZJSLAM__IMU_TYPE_HPP
#define ZJSLAM__IMU_TYPE_HPP

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>


// IMU (加速度计, 陀螺仪)
namespace IMU {


// 设备信息
class Device {

public:
    typedef std::shared_ptr<Device> Ptr;

    Eigen::DiagonalMatrix<float, 6> cov, covWalk;

    Device(float acc_noise, float acc_walk, float gyro_noise, float gyro_walk, float frequency) {
      float sf = sqrt(frequency);
      float na2 = pow(acc_noise * sf, 2), ng2 = pow(gyro_noise * sf, 2),
          wa2 = pow(acc_walk / sf, 2), wg2 = pow(gyro_walk / sf, 2);
      cov.diagonal() << ng2, ng2, ng2, na2, na2, na2;
      covWalk.diagonal() << wg2, wg2, wg2, wa2, wa2, wa2;
    }

    static Ptr fromYAML(YAML::Node node) {
      return Ptr(new Device(
          node["acc_noise"].as<float>(),
          node["acc_walk"].as<float>(),
          node["gyro_noise"].as<float>(),
          node["gyro_walk"].as<float>(),
          node["frequency"].as<float>()
      ));
    }
};


// 测量值
class Sample {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 线性加速度, 角速度
    Eigen::Vector3f a, w;

    Sample(const float &acc_x, const float &acc_y, const float &acc_z,
           const float &ang_vel_x, const float &ang_vel_y, const float &ang_vel_z
    ) : a(acc_x, acc_y, acc_z), w(ang_vel_x, ang_vel_y, ang_vel_z) {}

    Sample(const Eigen::Vector3f acc, const Eigen::Vector3f gyro) : a(acc), w(gyro) {}

    Sample(const cv::Point3f acc, const cv::Point3f gyro) : a(acc.x, acc.y, acc.z), w(gyro.x, gyro.y, gyro.z) {}
};
}

#endif
