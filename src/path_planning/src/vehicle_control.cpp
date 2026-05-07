/**
 * subscribe to /purePursuit/steering_angle 
 * publish to /carmaker/vehicle_control
 */

 #include "rclcpp/rclcpp.hpp"
 #include <std_msgs/msg/float64.hpp>
 #include <vehiclecontrol_msgs/msg/vehicle_control.hpp>

using std::placeholders::_1;

class VehicleCtrlNode : public rclcpp::Node
{
public:
    VehicleCtrlNode() : Node("vehicle_ctrl_node")
    {
        // subscribers
        steering_sub_ = create_subscription<std_msgs::msg::Float64>("/purePursuit/steering_angle", 10, std::bind(&VehicleCtrlNode ::steeringCallback, this, _1));

        // publishers
        vehCtrl_pub_ = create_publisher<vehiclecontrol_msgs::msg::VehicleControl>("/carmaker/vehicle_control", 10);
        
        // timer
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&VehicleCtrlNode::timerCallback, this));
 
    }
 
 private:
    // subscriber
     rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_sub_;

     // publisher
     rclcpp::Publisher<vehiclecontrol_msgs::msg::VehicleControl>::SharedPtr vehCtrl_pub_;
     rclcpp::TimerBase::SharedPtr timer_;

     // constants / variables
     double steering_;
     bool has_steering_ = false;

     // subscriber callbacks
     void steeringCallback(const std_msgs::msg::Float64::SharedPtr msg)
     {
         steering_ = msg->data;
         has_steering_ = true;
     }

    // timer callback
    void timerCallback(){
        if (!has_steering_){
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for steering command");
            return;
        };

        // command message to publish
        vehiclecontrol_msgs::msg::VehicleControl cmd; 

        // fill cmd message
        cmd.use_vc = true;

        cmd.selector_ctrl = 1;
        cmd.gas = 0.3;
        cmd.brake = 0.0;

        cmd.steer_ang = steering_ *  15 ;
        cmd.steer_ang_vel = 0.0;
        cmd.steer_ang_acc = 0.0;
        // publish
        vehCtrl_pub_->publish(cmd);
    }
 
 };

 
 int main(int argc, char * argv[])
 {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleCtrlNode>());
    rclcpp::shutdown();
    return 0;
 }
 
