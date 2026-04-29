#include <kf_node/kf_noise.hpp>

using std::placeholders::_1; 

// constructor
imuKF::imuKF(const std::string &name) : Node(name), initialized(false)
{

    // gain for Q and R
    declare_parameter<double>("Q", 0.01);
    declare_parameter<double>("R", 0.05);
    auto q = this->get_parameter("Q").get_value<double>();
    auto r = this->get_parameter("R").get_value<double>();

    x = Eigen::VectorXd::Zero(6); // x = [ax ay az wx wy wz]^T 
    A = Eigen::MatrixXd::Identity(6,6); 
    C = Eigen::MatrixXd::Identity(6,6); // direct measurement of all states
    P = Eigen::MatrixXd::Identity(6,6);
    Q = q * Eigen::MatrixXd::Identity(6, 6);
    R = r * Eigen::MatrixXd::Identity(6,6);
    

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/carmaker/imu", 10, std::bind(&imuKF::imu_callback, this, _1));
    kf_pub_ = create_publisher<sensor_msgs::msg::Imu>("/carmaker/kf_imu", 10);

    RCLCPP_INFO(this->get_logger(), "IMU Kalman Filter node started");
}

void imuKF::imu_callback(const sensor_msgs::msg::Imu &msg) {
    Eigen::VectorXd z(6);
    z << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, // [ax, ay, az]
         msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z; // [wx wy wz]


    if (!initialized){
        x = z; 
        initialized = true;
    }
    else {
        // prediction step
        Eigen::VectorXd x_pred = A * x; 
        Eigen::MatrixXd P_pred = A * P * A.transpose() + Q; 

        // update step
        Eigen::MatrixXd S = C * P_pred * C.transpose() + R; 
        Eigen::MatrixXd K = P_pred * C.transpose() * S.inverse();
        
        // state update using measurement z and C 
        x = x_pred + K * (z -  C * x_pred);

        // Error covariance 
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        P = (I - K * C) * P_pred * (I - K * C).transpose() + K * R * K.transpose();

    }

    // publish filtered messages
    sensor_msgs::msg::Imu out_msg = msg;
    out_msg.linear_acceleration.x = x(0); 
    out_msg.linear_acceleration.y = x(1); 
    out_msg.linear_acceleration.z = x(2); 
    out_msg.angular_velocity.x = x(3); 
    out_msg.angular_velocity.y = x(4);
    out_msg.angular_velocity.z = x(5);

    kf_pub_->publish(out_msg);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<imuKF>("imuKF_node"));
    rclcpp::shutdown();
    return 0;
}


