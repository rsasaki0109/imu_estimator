// Copyright (c) 2020, Ryohei Sasaki
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#ifndef IMU_ESTIMATOR__EKF_HPP_
#define IMU_ESTIMATOR__EKF_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

class EKFEstimator
{
public:
  EKFEstimator()
  : Cov_(Eigen::Matrix<double, 7, 7>::Identity() * 1.0),
    Q_(Eigen::Matrix3d::Identity() * 0.01),
    R_(Eigen::Matrix3d::Identity() * 0.033),
    gravity_vec_(Eigen::Vector3d(0.0, 0.0, 9.8067)),
    tau_gyro_bias_{1.0}
  {
    /* x =[qw qx qy qz gyro_bias_x gyro_bias_y gyro_bias_z] */
    x_ << 1, 0, 0, 0, 0, 0, 0;
  }

  void predictionUpdate(
    Eigen::Matrix<double, 7, 1> & predeted_x,
    Eigen::Matrix<double, 7, 7> & predeted_Cov,
    const Eigen::Vector3d & gyro)
  {
    Eigen::Vector4d quat = x_.head<4>();
    Eigen::Vector3d gyro_bias = x_.tail<3>();

    /* gyro2omega */
    Eigen::Vector3d w = gyro - gyro_bias;
    Eigen::Matrix4d omega;
    omega <<
      0, -w[0], -w[1], -w[2],
      w[0], 0, w[2], -w[1],
      w[1], -w[2], 0, w[0],
      w[2], w[1], -w[0], 0;

    /* predeted_x = f(x, w) */
    Eigen::Vector4d predeted_quat;
    predeted_quat = quat + dt_ / 2 * omega * quat;
    predeted_x.head<4>() = predeted_quat.normalized();
    predeted_x.tail<3>() = gyro_bias;

    /* predeted_Cov = F Cov_ Ft + L Q_ Lt */
    /* F */
    Eigen::Matrix<double, 7, 7> F;
    F = Eigen::Matrix<double, 7, 7>::Identity();
    F.block<4, 4>(0, 0) += dt_ / 2 * omega;
    F.block<4, 3>(0, 4) <<
      +quat[1], +quat[2], +quat[3],
      -quat[0], +quat[3], -quat[2],
      -quat[3], -quat[0], +quat[1],
      +quat[2], -quat[1], -quat[0];
    F.block<4, 3>(0, 4) *= dt_ / 2;
    F.block<3, 3>(3, 3) -= Eigen::Matrix<double, 3, 3>::Identity() * dt_ / tau_gyro_bias_;
    /* L */
    Eigen::Matrix<double, 7, 3> L;
    L <<
      -quat[1], -quat[2], -quat[3],
      +quat[0], -quat[3], +quat[2],
      +quat[3], +quat[0], -quat[1],
      -quat[2], +quat[1], +quat[0],
      0, 0, 0,
      0, 0, 0,
      0, 0, 0;
    L *= dt_ / 2;
    predeted_Cov = F * Cov_ * F.transpose() + L * Q_ * L.transpose();
  }

  void observationUpdate(
    const Eigen::Matrix<double, 7, 1> & predeted_x,
    const Eigen::Matrix<double, 7, 7> & predeted_Cov,
    const Eigen::Vector3d & z)
  {
    Eigen::Vector4d predeted_quat = predeted_x.head<4>();

    /* y = z - h(x) */
    Eigen::Vector3d acc;
    Eigen::Quaternion<double> quat_tmp(predeted_quat[0], predeted_quat[1],
      predeted_quat[2], predeted_quat[3]);
    acc = quat_tmp.conjugate()._transformVector(gravity_vec_);
    Eigen::Vector3d y = z - acc;

    /* H */
    Eigen::Matrix<double, 3, 7> H;
    double qw = predeted_quat[0], qx = predeted_quat[1],
      qy = predeted_quat[2], qz = predeted_quat[3];
    H <<
      -qy, +qz, -qw, +qx, 0, 0, 0,
      +qx, +qw, +qz, +qy, 0, 0, 0,
      +qw, -qx, -qy, +qz, 0, 0, 0;
    H *= 2 * gravity_vec_[2];

    /* x_ */
    Eigen::Matrix<double, 7, 3> K = predeted_Cov * H.transpose() *
      (H * predeted_Cov * H.transpose() + R_).inverse();
    Eigen::Matrix<double, 7, 1> x_tmp = predeted_x + K * y;
    x_.head<4>() = x_tmp.head<4>().normalized();
    x_.tail<3>() = x_tmp.tail<3>();

    /* Cov_ */
    Cov_ = (Eigen::Matrix<double, 7, 7>::Identity() - K * H) * predeted_Cov;
  }

  void filterOneStep(
    Eigen::Quaternion<double> & quat,
    const double dt,
    const Eigen::Vector3d & acc,
    const Eigen::Vector3d & gyro)
  {
    setdt(dt);

    Eigen::Matrix<double, 7, 1> predeted_x;
    Eigen::Matrix<double, 7, 7> predeted_Cov;
    predictionUpdate(predeted_x, predeted_Cov, gyro);

    observationUpdate(predeted_x, predeted_Cov, acc);

    Eigen::Quaternion<double> quat_tmp(x_[0], x_[1], x_[2], x_[3]);
    quat = quat_tmp;
  }

  void setdt(const double dt)
  {
    dt_ = dt;
  }

  void setProcessNoize(const double process_noize)
  {
    Q_ = Eigen::Matrix3d::Identity() * process_noize;
  }


  void setObservationNoize(const double observation_noize)
  {
    R_ = Eigen::Matrix3d::Identity() * observation_noize;
  }

private:
  Eigen::Matrix<double, 7, 1> x_;
  Eigen::Matrix<double, 7, 7> Cov_;
  double dt_;

  Eigen::Matrix3d Q_, R_;
  Eigen::Vector3d gravity_vec_;
  double tau_gyro_bias_;
};

#endif  // IMU_ESTIMATOR__EKF_HPP_
