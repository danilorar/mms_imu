#ifndef KF_NOISE_HPP
#define KF_NOISE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>


class imuKF : public rclcpp::Node
{
public:
    imuKF(const std::string &name);

private:    

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr kf_pub_;
    void imu_callback(const sensor_msgs::msg::Imu &msg);

    // KF states 
    Eigen::VectorXd x; // 6x1
    Eigen::MatrixXd A; // 6x6 
    Eigen::MatrixXd C; // 6x6
    Eigen::MatrixXd P; // 6x6
    Eigen::MatrixXd Q; // 6x6
    Eigen::MatrixXd R; // 6x6 

    bool initialized; 

};


#endif // KF_NOISE_HPP