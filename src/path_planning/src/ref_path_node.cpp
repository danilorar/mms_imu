/**
 * /reference_path 
 * nav_msgs/msg/Path
 * x = R cos(theta) and y = R sin(theta)
 * 2º create pure_persuit_node subscribe to /reference_path and publish to /cmd_vel
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

class ReferencePathNode : public rclcpp::Node
{
public:
    ReferencePathNode() : Node("referencePathNode")
    {
        // publisher
        path_pub_ = create_publisher<nav_msgs::msg::Path>("/referencePath", 10);

        // timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ReferencePathNode::timerCallback, this));

        RCLCPP_INFO(get_logger(), "Reference path node started");
    }

private:
    // publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    // timer
    rclcpp::TimerBase::SharedPtr timer_;

    // timer callback
    void timerCallback()
    {
        // create message to publish
        nav_msgs::msg::Path path_msg;

        // fill path_msg

        // header
        path_msg.header.stamp = now();    // now() == current time
        path_msg.header.frame_id = "map"; // frame of reference

        const double radius = 30.0;
        const double center_x = 100.0;
        const double center_y = 30.0;
        const int n_points = 200;

        // straight line path
        for (int i = 0; i < 300; ++i)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;

            pose.pose.position.x = static_cast<double>(i);
            pose.pose.position.y = 0.0;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;

            path_msg.poses.push_back(pose);
        }

        // publish
        path_pub_->publish(path_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReferencePathNode>());
    rclcpp::shutdown();
    return 0;
}
