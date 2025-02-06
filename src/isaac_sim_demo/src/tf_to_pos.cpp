#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>

using namespace std::chrono_literals;

class TransformListenerNode : public rclcpp::Node
{
public:
    TransformListenerNode()
        : Node("transform_listener_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {

        this->declare_parameter("Quad_Number", "0");
        std::string quad_num = this->get_parameter("Quad_Number").as_string();

        pub_drone = this->create_publisher<geometry_msgs::msg::PoseStamped>("/Quadrotor_"+quad_num+"/goal_pose", 10);

        timer_ = this->create_wall_timer(
            500ms, std::bind(&TransformListenerNode::timer_callback, this));
    }

private:

    void timer_callback()
    {
        std::string quadrotor_frame = "Drone"+this->get_parameter("Quad_Number").as_string();

        publish_pose("base_link", quadrotor_frame, pub_drone);

    }

    void publish_pose(const std::string &from_frame, const std::string &to_frame, const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &publisher)
    {
        try
        {   
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform(from_frame, to_frame, tf2::TimePointZero);
            
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = from_frame;
            pose.pose.position.x = transform_stamped.transform.translation.x;
            pose.pose.position.y = transform_stamped.transform.translation.y;
            pose.pose.position.z = transform_stamped.transform.translation.z;
            pose.pose.orientation = transform_stamped.transform.rotation;

            publisher->publish(pose);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", from_frame.c_str(), to_frame.c_str(), ex.what());
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_drone;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformListenerNode>());
    rclcpp::shutdown();
    return 0;
}
