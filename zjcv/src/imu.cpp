#include "utils/glog.hpp"
#include "zjcv/imu.hpp"

namespace IMU {


void Preintegration::reset(double tStart) {
  mtStart = mtEnd = tStart;
  it = 0;
  iP.setZero();
  iV.setZero();
  iTheta.setZero();
  iR = Sophus::SO3f();

  Jpa.setZero();
  Jpg.setZero();
  Jva.setZero();
  Jvg.setZero();
  JRg.setZero();
}


void Preintegration::integrate(const double &tCurframe, const std::vector<double> &vTimestamp, const std::vector<Sample> &vSample) {
  ASSERT(vTimestamp.size() == vSample.size(), "Preintegration: The size of vTimestamp and vSample must be equal")
  size_t n = vTimestamp.size();
  if (n > 0) {
    ASSERT(mtEnd <= vTimestamp[0] && vTimestamp[n - 1] <= tCurframe,
           "Preintegration: timestamp must be in ascending order")
    if (n == 1) {
      integrate(tCurframe - mtEnd, vSample[0]);
    } else {
      // 初始段
      {
        double t0 = vTimestamp[0], t1 = vTimestamp[1];
        const Sample &s0 = vSample[0], &s1 = vSample[1];
        Sample interp = s0 + (s0 - s1) * static_cast<float>((mtEnd - t0) / (t0 - t1) / 2);
        integrate(t0 - mtEnd, interp);
      }
      // 中间段
      for (int i = 0; i < n - 1; ++i) {
        integrate(vTimestamp[i + 1] - vTimestamp[i], (vSample[i + 1] + vSample[i]) * 0.5f);
      }
      // 结束段
      {
        double t0 = vTimestamp[n - 1], t1 = vTimestamp[n - 2];
        const Sample &s0 = vSample[n - 1], &s1 = vSample[n - 2];
        Sample interp = s0 + (s0 - s1) * static_cast<float>((tCurframe - t0) / (t0 - t1) / 2);
        integrate(tCurframe - t0, interp);
      }
    }
  }
  mtEnd = tCurframe;
}


void Preintegration::integrate(const double &dt, const Sample &sample) {
  Sample M = sample - B, dM = M * dt;
  const Eigen::Vector3f &dV = dM.a, &dTheta = dM.w, dV_rot = iR.matrix() * dV;
  // 更新雅可比矩阵
  Eigen::Matrix3f a_hat = Sophus::SO3f::hat(dTheta).matrix();
  Eigen::Matrix3f tmp = iR.matrix() * dt;
  Jva -= tmp;
  Jpa += Jva * dt - 0.5f * dt * tmp;
  tmp *= a_hat * JRg;
  Jvg -= tmp;
  Jpg += Jvg * dt - 0.5f * dt * tmp;
  // 旋转积分
  Eigen::Matrix3f deltaR = Sophus::SO3f::exp(dTheta).matrix();
  Eigen::Matrix3f rightJ = Eigen::Matrix3f::Identity() * 2 - Sophus::SO3f::leftJacobian(dTheta);
  JRg = deltaR.transpose() * JRg - rightJ * dt;
  // 更新积分值
  it += dt;
  iP += (iV + 0.5f * dV_rot) * dt;
  iV += dV_rot;
  iTheta += dTheta;
  iR = Sophus::SO3f::exp(iTheta);
}


void MovingPose::predict_from(MovingPose &prev, Preintegration &preint, bool update) {
  Sample deltaB = (update) ? preint.B : prev.B - preint.B;
  double &it = preint.it;
  Eigen::Vector3f dv(0, 0, -9.81 * it);

  Sophus::SO3f &R0 = prev.T_world_imu.so3();
  Eigen::Vector3f &t0 = prev.T_world_imu.translation();
  Eigen::Vector3f &v0 = prev.v;

  B = prev.B;
  T_world_imu.so3() = R0 * preint.iR * Sophus::SO3f::exp(preint.JRg * deltaB.w);
  T_world_imu.translation() = t0 + v0 * it + 0.5f * dv * it + R0 * (preint.iP + preint.Jpa * deltaB.a + preint.Jpg * deltaB.w);
  v = v0 + dv + R0 * (preint.iV + preint.Jva * deltaB.a + preint.Jvg * deltaB.w);
}

}
