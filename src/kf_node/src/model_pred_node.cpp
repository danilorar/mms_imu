#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Dense>
#include <cmath>

#include <bicycle_model.hpp>
using std::placeholders::_1;

class ModelPredNode : public rclcpp::Node
{

public:
    ModelPredNode() : Node("model_pred_node")
    {

        // subscriber: vx, steering
        vx_sub_ = create_subscription<std_msgs::msg::Float64>("/carmaker/speed", 10, std::bind(&ModelPredNode::vxCallback,this, _1));
        steering_sub_ = create_subscription<std_msgs::msg::Float64>("/carmaker/steering_angle", 10, std::bind(&ModelPredNode::steeringCallback, this, _1));

        // publishers: yaw_rate, beta (side slip), by
        vy_pub_ = create_publisher<std_msgs::msg::Float64>("/model_prediction/vy", 10);
        yaw_rate_pub_ = create_publisher<std_msgs::msg::Float64>("/model_prediction/yaw_rate", 10);
        beta_pub_ = create_publisher<std_msgs::msg::Float64>("/model_prediction/beta", 10);
        ay_pub_ = create_publisher<std_msgs::msg::Float64>("/model_prediction/ay", 10);

        // timer
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&ModelPredNode::timerCallback, this));

        x_.setZero();

        RCLCPP_INFO(get_logger(), "Model predictor node started");
        
    }

private:
    void vxCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        vx_ = msg->data;
        has_vx_ = true;
    }

    void steeringCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        delta_ = msg->data;
        has_delta_ = true;
    }

    // function that executes the prediction every 10 miliseconds 
    void timerCallback()
    {
        if (!has_vx_ || !has_delta_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Waiting for vx and steering_angle topics...");
            return;
        }

        const double dt = 0.01;

        // Bicycle model
        Eigen::Matrix2d A = bicycle_model_.computeA(vx_); 
        Eigen::Vector2d B = bicycle_model_.computeB();

        // apply steering ratio to get the wheel angle from the steering angle 
        const double steering_ratio = 15.0;
        const double delta_model = delta_ / steering_ratio;

        Eigen::Vector2d x_dot = A * x_ + B * delta_model;

        double const ay_model = -(x_dot(0) + vx_ * x_(1));

        x_ = x_ + dt * x_dot;

        const double vy = x_(0);
        const double yaw_rate = x_(1);
        const double vx_safe = std::max(std::abs(vx_), 1.0);
        const double beta = std::atan2(vy, vx_safe);

        // prepare msg to publish
        std_msgs::msg::Float64 vy_msg;
        std_msgs::msg::Float64 yaw_rate_msg;
        std_msgs::msg::Float64 beta_msg;
        std_msgs::msg::Float64 ay_msg;

        vy_msg.data = vy;
        yaw_rate_msg.data = yaw_rate;
        beta_msg.data = beta;
        ay_msg.data = ay_model;

        // publish
        vy_pub_->publish(vy_msg);
        yaw_rate_pub_->publish(yaw_rate_msg);
        beta_pub_->publish(beta_msg);
        ay_pub_->publish(ay_msg);
    }

    // declare model and state variables
    BicycleModel bicycle_model_;

    Eigen::Vector2d x_{Eigen::Vector2d::Zero()};

    double vx_ = 0.0;
    double delta_ = 0.0;

    bool has_vx_ = false;
    bool has_delta_ = false;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vx_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vy_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_rate_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr beta_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ay_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModelPredNode>());
    rclcpp::shutdown();
    return 0;
}