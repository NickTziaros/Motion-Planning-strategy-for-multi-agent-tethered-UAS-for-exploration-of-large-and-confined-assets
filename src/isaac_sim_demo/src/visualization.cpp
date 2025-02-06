#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "custom_interfaces/srv/planner_workspace.hpp"




class VizNode : public rclcpp::Node {
public:
  VizNode() : Node("viz_node") {
    // Create a publisher for the marker
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("planner_ws", 10);
    
    // Create the service
    service_ = this->create_service<custom_interfaces::srv::PlannerWorkspace>(
      "ws_marker_pub", std::bind(&VizNode::publish_planner_ws, this, std::placeholders::_1, std::placeholders::_2));
  
  
  
  }

private:
  void publish_planner_ws(const std::shared_ptr<custom_interfaces::srv::PlannerWorkspace::Request> request,
                             std::shared_ptr<custom_interfaces::srv::PlannerWorkspace::Response> response) {
    // Create a marker message

    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "World";
    marker.header.stamp = this->now();
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = (request->upper+request->lower)/2;
    marker.pose.position.y = (request->upper+request->lower)/2;
    marker.pose.position.z = (request->upper+request->lower)/2;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = abs(request->upper)+abs(request->lower);
    marker.scale.y = abs(request->upper)+abs(request->lower);
    marker.scale.z = abs(request->upper)+abs(request->lower);
    marker.color.a = 0.3;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.2;
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Success");

    // Publish the marker
    publisher_->publish(marker);
    response->success = true; // Indicate that the marker was published successfully
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::Service<custom_interfaces::srv::PlannerWorkspace>::SharedPtr service_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VizNode>());
  rclcpp::shutdown();
  return 0;
}
