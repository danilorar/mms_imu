/**
 * subscribe to referencePath and /carmaker/odometry
 * publish to /purePursuit/steering_angle as geometry_msgs/msg/PointStamped
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h> // for yaw
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/float64.hpp>
#include <limits>
#include <cmath>

using std::placeholders::_1;

class PurePursuitNode:public rclcpp::Node
{
public:
    PurePursuitNode() : Node("purePursuitNode")
    {
        // subscribers
        using std::placeholders::_1;
        path_sub_ = create_subscription<nav_msgs::msg::Path>("/referencePath", 10, std::bind(&PurePursuitNode::pathCallback, this, _1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/carmaker/odometry", 10, std::bind(&PurePursuitNode::odomCallback, this, _1));

        // publishers 
        ppc_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/purePursuit/lookahead_point", 10);
        steering_pub_ = create_publisher<std_msgs::msg::Float64>("/purePursuit/steering_angle", 10);

            // timers
        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10),
                std::bind(&PurePursuitNode::timerCallback, this));

        RCLCPP_INFO(get_logger(), "Pure Pursuit node started");
    }

private:
    // subscribers 
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // publishers 
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ppc_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // variables 
    nav_msgs::msg::Path path_; 
    nav_msgs::msg::Odometry odom_;
    bool has_path_ = false; 
    bool has_odom_ = false; 

    // vehicle variables 
    double vehicle_x_ = 0.0; 
    double vehicle_y_ = 0.0; 
    double vehicle_yaw_ = 0.0; // yaw

    // timer Callback 
    void timerCallback()
    {
        // warning
        if (!has_path_ || !has_odom_){
            RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Waiting for reference path and odometry...");
            return;
        }

        // functions
        const int closest_index = findClosestPathIndex();
        const int lookahead_index = findLookaheadIndex(closest_index);

        // point msg 
        geometry_msgs::msg::PointStamped point_msg_;
        std_msgs::msg::Float64 steering_msg;

        // fill point_msg_
        point_msg_.header.stamp = now(); // current time
        point_msg_.header.frame_id = "map";

        // point position x,y,z
        point_msg_.point.x = path_.poses[lookahead_index].pose.position.x;;
        point_msg_.point.y = path_.poses[lookahead_index].pose.position.y;
        point_msg_.point.z = 0;

        // compute steering 
        const double target_x = path_.poses[lookahead_index].pose.position.x;
        const double target_y = path_.poses[lookahead_index].pose.position.y;

        const double dx = target_x - vehicle_x_;
        const double dy = target_y - vehicle_y_;

        const double target_angle = std::atan2(dy, dx);
        const double alpha = target_angle - vehicle_yaw_;

        const double wheelbase = 2.9845; // [m]
        const double lookahead_distance = std::sqrt(dx * dx + dy * dy);

        const double delta = std::atan2(
            2.0 * wheelbase * std::sin(alpha),
            lookahead_distance);

        // debug:
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "closest_index=%d, lookahead_index=%d, vehicle=(%.2f, %.2f), target=(%.2f, %.2f)",
            closest_index,
            lookahead_index,
            vehicle_x_,
            vehicle_y_,
            point_msg_.point.x,
            point_msg_.point.y);

        steering_msg.data = delta;

        // publish msgs
        ppc_pub_->publish(point_msg_);
        steering_pub_->publish(steering_msg);
        }

    // subscribers callback
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path_ = *msg; // copy the message content to the member variable
        has_path_ = true; 
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        vehicle_x_ = msg->pose.pose.position.x; 
        vehicle_y_ = msg->pose.pose.position.y;
        vehicle_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
        has_odom_ = true; 
    }

    // pure persuit 
    // look for path function
    int findClosestPathIndex()
    {
        int closest_index = 0;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < path_.poses.size(); ++i)
        {
            const double px = path_.poses[i].pose.position.x;
            const double py = path_.poses[i].pose.position.y;

            const double dx = px - vehicle_x_;
            const double dy = py - vehicle_y_;
            const double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < min_dist)
            {
                min_dist = dist;
                closest_index = static_cast<int>(i);
            }
        }

        return closest_index;
    }

    // find first point at lookahead distance = 30.0
    int findLookaheadIndex(int closest_index)
    {
        const double lookahead_distance = 10.0;
        double accumulated_distance = 0.0;

        const int n = static_cast<int>(path_.poses.size());

        for (int step = 0; step < n; ++step)
        {
            int i = (closest_index + step) % n;
            int j = (closest_index + step + 1) % n;

            const double x_i = path_.poses[i].pose.position.x;
            const double y_i = path_.poses[i].pose.position.y;

            const double x_j = path_.poses[j].pose.position.x;
            const double y_j = path_.poses[j].pose.position.y;

            const double dx = x_j - x_i;
            const double dy = y_j - y_i;

            accumulated_distance += std::sqrt(dx * dx + dy * dy);

            if (accumulated_distance >= lookahead_distance)
            {
                return j;
            }
        }

        return closest_index;
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
