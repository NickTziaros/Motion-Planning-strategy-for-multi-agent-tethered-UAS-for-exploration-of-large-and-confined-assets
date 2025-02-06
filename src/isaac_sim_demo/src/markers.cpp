#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <cmath>

using namespace std::chrono_literals;

class CircleMarkerPublisher : public rclcpp::Node
{
public:
    CircleMarkerPublisher() : Node("circle_marker_publisher")
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("Quadrotor1_marker", 10);
        timer_ = this->create_wall_timer(200ms, std::bind(&CircleMarkerPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto marker = std::make_unique<visualization_msgs::msg::Marker>();
        marker->header.frame_id = "World";
        marker->header.stamp = this->get_clock()->now();
        marker->type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker->action = visualization_msgs::msg::Marker::ADD;
        marker->pose.orientation.w = 1.0;
        marker->scale.x = 0.1;
        marker->scale.y = 0.1;
        marker->scale.z = 0.1;
        marker->color.a = 1.0;
        marker->color.r = 1.0;
        marker->color.g = 1.0;
        marker->color.b = 0.0;



        publisher_->publish(std::move(marker));
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircleMarkerPublisher>());
    rclcpp::shutdown();
    return 0;
}