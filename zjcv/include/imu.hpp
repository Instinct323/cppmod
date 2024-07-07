#ifndef ZJCV__IMU_HPP
#define ZJCV__IMU_HPP

#include <Eigen/Core>
#include <memory>
#include <opencv2/opencv.hpp>
#include <Sophus/so3.hpp>
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

    Device(const Device &) = delete;

    static Ptr from_yaml(const YAML::Node &node) {
      if (node.IsNull()) return nullptr;
      return std::make_shared<Device>(
          node["acc_noise"].as<float>(),
          node["acc_walk"].as<float>(),
          node["gyro_noise"].as<float>(),
          node["gyro_walk"].as<float>(),
          node["frequency"].as<float>()
      );
    }
};


// 测量值
class Sample {

public:
    // 线性加速度, 角速度
    const Eigen::Vector3f a, w;

    Sample() : a(0, 0, 0), w(0, 0, 0) {}

    Sample(const float &acc_x, const float &acc_y, const float &acc_z,
           const float &angvel_x, const float &angvel_y, const float &angvel_z
    ) : a(acc_x, acc_y, acc_z), w(angvel_x, angvel_y, angvel_z) {}

    Sample(const Eigen::Vector3f &acc, const Eigen::Vector3f &gyro) : a(acc), w(gyro) {}

    // 运算方法
    Sample operator+(const Sample &other) const { return {a + other.a, w + other.w}; }

    Sample operator-(const Sample &other) const { return {a - other.a, w - other.w}; }

    Sample operator*(const float &scale) const { return {a * scale, w * scale}; }

    Sample operator*(const Sample &other) const { return {a.cwiseProduct(other.a), w.cwiseProduct(other.w)}; }

    Sample operator/(const float &scale) const { return {a / scale, w / scale}; }

    Sample operator/(const Sample &other) const { return {a.cwiseQuotient(other.a), w.cwiseQuotient(other.w)}; }
};


// 预积分
class Preintegration {
    Device *mpDevice;
    Sample mBias;

public:
    typedef std::shared_ptr<Preintegration> Ptr;

    double mtLastframe, it;
    Eigen::Vector3f iP, iV, iTheta;
    Sophus::SO3f iR;

    explicit Preintegration(Device *pDevice, double tStart, Sample bias = Sample()
    ) : mpDevice(pDevice), mBias(std::move(bias)) { reset(tStart); }

    Preintegration(const Preintegration &) = delete;

    void reset(double tStart);

    // 测量值
    void integrate(const double &tCurframe, const std::vector<double> &vTimestamp, const std::vector<Sample> &vSample);

private:
    void integrate(const double &dt, const Sample &sample);
};

}

#endif
