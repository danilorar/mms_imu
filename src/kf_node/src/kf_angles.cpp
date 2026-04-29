#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <cmath>

using std::placeholders::_1;

class ImuFusionNode : public rclcpp::Node
{
public:
    ImuFusionNode() : Node("imu_fusion_node")
    {
        // parameter
        this->declare_parameter<double>("alpha", 0.98);
        auto alpha_ = this->get_parameter("alpha").get_value<double>();

        using std::placeholders::_1;
        sub_  = this->create_subscription<sensor_msgs::msg::Imu>("/carmaker/kf_imu", 10, std::bind(&ImuFusionNode::kf_callback, this, _1));
        pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/carmaker/imu_rpy", 10);

        RCLCPP_INFO(get_logger(), "IMU Fusion node started");
    }

private:

// subscriber callback 
void kf_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received message");

    double t = rclcpp::Time(msg->header.stamp).seconds();

    // linear acceleration 
    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y; 
    double az = msg->linear_acceleration.z; 

    // angular velocity  
    double wx = msg->angular_velocity.x; 
    double wy = msg->angular_velocity.y; 
    double wz = msg->angular_velocity.z; 

    // first 
    if (!initialize){
        last_time = t; 

        roll_ = std::atan2(ay, az);
        pitch_ = std::atan2(-ax, std::sqrt(ay*ay + az*az));
        yaw_= 0; 

        initialize = true; 
        return; 
    }

    double dt = t - last_time;  // delta to integrate
    last_time = t; 

    if (dt <= 0 || dt > 1.0)
        return;


    // gyro angles (integrate angular velocities)
    // prediction
    double roll_gyro = roll_ + wx * dt;
    double pitch_gyro = pitch_ + wy * dt; 
    double yaw_gyro = yaw_ + wz * dt; 

    // accelerometer angles
    // estimate
    double roll_acc = std::atan2(ay, az);
    double pitch_acc = std::atan2(-ax, std::sqrt(ay * ay + az * az));

    // sensor fuse
    roll_ = alpha_ * roll_gyro + (1 - alpha_) * roll_acc;
    pitch_ = alpha_ * pitch_gyro + (1 - alpha_) * pitch_acc;
    yaw_ = yaw_gyro;


    // publish in Vector3Stamped
    /**
     * float64 x
     * float64 y
     * float64 z
     */

    out_msg_.header = msg->header;
    out_msg_.vector.x = roll_;
    out_msg_.vector.y = pitch_;
    out_msg_.vector.z = yaw_;

   pub_->publish(out_msg_);
};

// declare 
rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_;
geometry_msgs::msg::Vector3Stamped out_msg_; 

bool initialize = false; 
double last_time = 0.0; 

// init assumption
double roll_ = 0.0;
double pitch_ = 0.0;
double yaw_ = 0.0;
double alpha_ = 0.98;

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuFusionNode>());
    rclcpp::shutdown();
    return 0;
}

