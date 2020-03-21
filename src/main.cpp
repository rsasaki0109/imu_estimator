#include <imu_estimator/ekf.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

#include <sensor_msgs/msg/imu.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

EKFEstimator ekf;

double previous_time_{-1};

void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg_ptr){
    
    double current_time_odom = msg_ptr->header.stamp.sec 
                                    + msg_ptr->header.stamp.nanosec * 1e-9;
    if(previous_time_ == -1){
        previous_time_ = current_time_odom;
        return;
    }
    double dt_imu = current_time_odom - previous_time_;
    previous_time_ = current_time_odom; 

    Eigen::Quaternion<double> quat;
    Eigen::Vector3d gyro{msg_ptr->angular_velocity.z, msg_ptr->angular_velocity.y, msg_ptr->angular_velocity.x};
    Eigen::Vector3d acc{msg_ptr->linear_acceleration.x,msg_ptr->linear_acceleration.y,msg_ptr->linear_acceleration.z};

    ekf.filterOneStep(quat, dt_imu, acc, gyro);

    double roll, pitch, yaw;
    Eigen::Vector3d rpy;
    tf2::Quaternion previous_quat_tf(quat.x(), quat.y(), quat.z(), quat.w());
    tf2::Matrix3x3(previous_quat_tf).getRPY(roll, pitch, yaw);
    rpy(0) = roll; 
    rpy(1) = pitch;
    rpy(2) = yaw;

    std::cout << "----" << std::endl;
    std::cout << "dt[s]:" << dt_imu << std::endl;
    std::cout << "roll[deg]:" << rpy(0) * 180 / M_PI << std::endl;
    std::cout << "pitch[deg]:" << rpy(1) * 180 / M_PI << std::endl;
    std::cout << "yaw[deg]:" << rpy(2) * 180 / M_PI << std::endl;

}

int main(int argc, char * argv[]){
    using namespace std::chrono_literals;

    ekf.setProcessNoize(0.033);
    ekf.setObservationNoize(0.033);

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("imu_estimator");

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_ = 
        node->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", rclcpp::SensorDataQoS(),
         std::bind(imuCallback, std::placeholders::_1)
         );
  
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}