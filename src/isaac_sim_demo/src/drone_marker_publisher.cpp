#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>
class DroneMarkerPublisher : public rclcpp::Node
{
public:
    DroneMarkerPublisher()
        : Node("drone_marker_publisher"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("drone_markers", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200), std::bind(&DroneMarkerPublisher::publishMarkers, this));
    }

private:
    void publishMarkers()
    {
        std::vector<std::string> drone_frames = {"Drone1", "Drone2", "Drone3", "Drone4"};
        visualization_msgs::msg::MarkerArray marker_array;
        int marker_id = 0;

        // Point at the origin (0, 0, 0)
        geometry_msgs::msg::Point origin_point;
        origin_point.x = 0.0;
        origin_point.y = 0.0;
        origin_point.z = 0.0;

        // Iterate over the drones and create line markers
        for (size_t i = 0; i < drone_frames.size(); ++i)
        {
            geometry_msgs::msg::Point drone_point;

            // Attempt to lookup the transform from the world frame to the drone frame
            try
            {
                geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform("base_link", drone_frames[i], tf2::TimePointZero);

                drone_point.x = transform.transform.translation.x;
                drone_point.y = transform.transform.translation.y;
                drone_point.z = transform.transform.translation.z;
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Could not transform %s: %s", drone_frames[i].c_str(), ex.what());
                continue;  // Skip this iteration if the transform fails
            }

            // Create a line marker
            visualization_msgs::msg::Marker line_marker;
            line_marker.header.frame_id = "base_link";
            line_marker.header.stamp = this->get_clock()->now();
            line_marker.ns = "drone_lines";
            line_marker.id = marker_id++;
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;

            // Line properties
            line_marker.scale.x = 0.01;  // Line width
            line_marker.color.r = 1.0;
            line_marker.color.g = 0.0;
            line_marker.color.b = 0.0;
            line_marker.color.a = 1.0;  // Fully opaque

            // Add the points to the line strip: previous drone point and current drone point
            if (i == 0)
            {
                // First line: connect origin to Drone1
                line_marker.points.push_back(origin_point);
            }
            else
            {
                // Subsequent lines: connect previous drone to current drone
                geometry_msgs::msg::Point prev_point = marker_array.markers.back().points.back();
                line_marker.points.push_back(prev_point);
            }
            line_marker.points.push_back(drone_point);

            // Add the marker to the array
            marker_array.markers.push_back(line_marker);
        }

        // Publish the marker array
        marker_pub_->publish(marker_array);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneMarkerPublisher>());
    rclcpp::shutdown();
    return 0;
}