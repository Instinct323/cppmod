#include "utils/glog.hpp"
#include "utils/sophus.hpp"
#include "zjcv/imu.hpp"

namespace IMU {


void Preintegration::reset(double tStart) {
  mtStart = mtEnd = tStart;
  it = 0;
  iP.setZero();
  iV.setZero();
  iR = Sophus::SO3f();

  Jpa.setZero();
  Jpw.setZero();
  Jva.setZero();
  Jvw.setZero();
  JRw.setZero();
}


void Preintegration::integrate(const double &tCurframe, const std::vector<double> &vTimestamp, const std::vector<Sample> &vSample) {
  ASSERT(vTimestamp.size() == vSample.size(), "Preintegration: The size of vTimestamp and vSample must be equal")
  size_t n = vTimestamp.size();
  if (n > 0) {
    ASSERT(mtEnd <= vTimestamp[0] && vTimestamp[n - 1] <= tCurframe,
           "Preintegration: timestamp must be in ascending order")
    int cnt = mMeasurements.size();
    if (n == 1) {
      mMeasurements.emplace_back(tCurframe - mtEnd, vSample[0]);
    } else {
      mMeasurements.reserve(cnt + n + 1);
      // 初始段
      {
        double t0 = vTimestamp[0], t1 = vTimestamp[1];
        const Sample &s0 = vSample[0], &s1 = vSample[1];
        Sample interp = s0 + (s0 - s1) * static_cast<float>((mtEnd - t0) / (t0 - t1) / 2);
        mMeasurements.emplace_back(t0 - mtEnd, interp);
      }
      // 中间段
      for (int i = 0; i < n - 1; ++i) {
        mMeasurements.emplace_back(vTimestamp[i + 1] - vTimestamp[i], (vSample[i + 1] + vSample[i]) * 0.5f);
      }
      // 结束段
      {
        double t0 = vTimestamp[n - 1], t1 = vTimestamp[n - 2];
        const Sample &s0 = vSample[n - 1], &s1 = vSample[n - 2];
        Sample interp = s0 + (s0 - s1) * static_cast<float>((tCurframe - t0) / (t0 - t1) / 2);
        mMeasurements.emplace_back(tCurframe - t0, interp);
      }
    }
    for (auto iter = mMeasurements.begin() + cnt; iter != mMeasurements.end(); ++iter) integrate(iter->first, iter->second);
  }
  mtEnd = tCurframe;
}


void Preintegration::integrate(const double &dt, const Sample &sample) {
  Sample M = sample - B, dM = M * dt;
  const Eigen::Vector3f &dV = dM.a, &dTheta = dM.w, dV_rot = iR.matrix() * dV;
  // 更新雅可比矩阵
  Eigen::Matrix3f JaR = Sophus::SO3f::hat(dTheta);
  Eigen::Matrix3f tmp = iR.matrix() * dt;
  Jpa += Jva * dt - 0.5f * dt * tmp;
  Jva -= tmp;
  tmp *= JaR * JRw;
  Jpw += Jvw * dt - 0.5f * dt * tmp;
  Jvw -= tmp;
  // 旋转积分
  Sophus::SO3f deltaR = Sophus::SO3f::exp(dTheta);
  Eigen::Matrix3f rightJ = Sophus::rightJacobian(dTheta);
  JRw = deltaR.matrix().transpose() * JRw - rightJ * dt;
  // 更新积分值
  it += dt;
  iP += (iV + 0.5f * dV_rot) * dt;
  iV += dV_rot;
  iR = deltaR * iR;
}


void MovingPose::predict_from(MovingPose &prev, Preintegration *preint, bool update) {
  Sample deltaB = (update) ? preint->B : prev.B - preint->B;
  float it = preint->it;
  Eigen::Vector3f iV(0, 0, -9.81f * it);

  Sophus::SO3f &R0 = prev.T_world_imu.so3();
  Eigen::Vector3f &t0 = prev.T_world_imu.translation();
  Eigen::Vector3f &v0 = prev.v;

  B = prev.B;
  T_world_imu.so3() = R0 * preint->iR * Sophus::SO3f::exp(preint->JRw * deltaB.w);
  T_world_imu.translation() = t0 + v0 * it + 0.5f * it * iV + R0 * (preint->iP + preint->Jpa * deltaB.a + preint->Jpw * deltaB.w);
  T_imu_world = T_world_imu.inverse();
  v = v0 + iV + R0 * (preint->iV + preint->Jva * deltaB.a + preint->Jvw * deltaB.w);
}

}
