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
#include <imu_estimator/ekf.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>

EKFEstimator ekf;

double previous_time_{-1};
std::string node_name_{"imu_estimator"};

void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg_ptr)
{
  double current_time_odom = msg_ptr->header.stamp.sec +
    msg_ptr->header.stamp.nanosec * 1e-9;
  if (previous_time_ == -1) {
    RCLCPP_INFO(
      rclcpp::get_logger(node_name_),
      "Subscribed first message");
    previous_time_ = current_time_odom;
    return;
  }
  double dt_imu = current_time_odom - previous_time_;
  previous_time_ = current_time_odom;

  Eigen::Quaternion<double> quat;
  Eigen::Vector3d gyro{msg_ptr->angular_velocity.z,
    msg_ptr->angular_velocity.y,
    msg_ptr->angular_velocity.x};
  Eigen::Vector3d acc{msg_ptr->linear_acceleration.x,
    msg_ptr->linear_acceleration.y,
    msg_ptr->linear_acceleration.z};

  ekf.filterOneStep(quat, dt_imu, acc, gyro);

  double roll, pitch, yaw;
  Eigen::Vector3d rpy;

  // convert to rpy according to ROS
  tf2::Quaternion tf2_quat;
  tf2::fromMsg(msg_ptr->orientation, tf2_quat);
  tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, yaw);

  rpy(0) = roll;
  rpy(1) = pitch;
  rpy(2) = yaw;

  RCLCPP_INFO(
    rclcpp::get_logger(node_name_),
    "roll[deg]: %f, pitch[deg]: %f, yaw[deg]: %f",
    rpy(0) * 180 / M_PI, rpy(1) * 180 / M_PI, rpy(2) * 180 / M_PI);
}

int main(int argc, char * argv[])
{
  using namespace std::chrono_literals;

  ekf.setProcessNoize(0.033);
  ekf.setObservationNoize(0.033);

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(node_name_);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_ =
    node->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SensorDataQoS(),
    std::bind(imuCallback, std::placeholders::_1)
    );

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
