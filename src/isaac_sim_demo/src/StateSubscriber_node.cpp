#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class QuadrotorStateSubscriber : public rclcpp::Node
{


    
public:
    QuadrotorStateSubscriber()
    : Node("quadrotor_state_subscriber")
    {
        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "Quadrotor_1/state/pose", 10, std::bind(&QuadrotorStateSubscriber::pose_callback, this, std::placeholders::_1));

        twist_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "Quadrotor_1/state/twist", 10, std::bind(&QuadrotorStateSubscriber::twist_callback, this, std::placeholders::_1));
    }




private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received PoseStamped message:\n Position: (%f, %f, %f)\n Orientation: (%f, %f, %f, %f)",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                    msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    }

    void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received TwistStamped message:\n Linear: (%f, %f, %f)\n Angular: (%f, %f, %f)",
                    msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
                    msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);
    }



    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscriber_;
};








int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QuadrotorStateSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
    