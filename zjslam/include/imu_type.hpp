#ifndef ZJSLAM__IMU_TYPE_HPP
#define ZJSLAM__IMU_TYPE_HPP

#include <Eigen/Core>
#include <opencv2/opencv.hpp>


namespace IMU {

    /**
     * @brief IMU 测量值 (加速度计, 陀螺仪)
     */
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
