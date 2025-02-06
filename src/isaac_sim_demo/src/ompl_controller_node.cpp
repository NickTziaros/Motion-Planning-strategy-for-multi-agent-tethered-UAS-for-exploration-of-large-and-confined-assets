#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp> 
#include <visualization_msgs/msg/marker.hpp>

#include "isaac_sim_demo/RRT_controller.hpp"
#include <Eigen/Dense>
#include "std_srvs/srv/trigger.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;


class PathPublisher : public rclcpp::Node
{
  public:
    PathPublisher()
    : Node("path_publisher")
    {
      this->declare_parameter("Quadrotor", "Quadrotor_0");
      
      // Create workspace service client
      client_ = this->create_client<custom_interfaces::srv::PlannerWorkspace>("ws_marker_pub");
      
      // get parameters
      std::string Quadrotor_num = this->get_parameter("Quadrotor").as_string();
      
      publisher_ = this->create_publisher<nav_msgs::msg::Path>("/"+Quadrotor_num+"/path_topic", 10);
      
      pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      ""+Quadrotor_num+"/state/pose", 10, std::bind(&PathPublisher::pose_callback, this, std::placeholders::_1));
      goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/"+Quadrotor_num+"/goal_pose", 10, std::bind(&PathPublisher::goal_callback, this, std::placeholders::_1));
      
      // init timer
      timer_ = this->create_wall_timer(
      800ms, std::bind(&PathPublisher::timer_callback,this));
    }

    rclcpp::Client<custom_interfaces::srv::PlannerWorkspace>::SharedPtr client_;



  private:

    std::vector<double> quad_pose = std::vector<double>(7);
    std::vector<double> goal_pose = std::vector<double>(7);
    RRT_controller::ompl_controller planner_;

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        quad_pose[0]= msg->pose.position.x;
        quad_pose[1]= msg->pose.position.y;
        quad_pose[2]= msg->pose.position.z;
        quad_pose[3]= msg->pose.orientation.x;
        quad_pose[4]= msg->pose.orientation.y;
        quad_pose[5]= msg->pose.orientation.z;
        quad_pose[6]= msg->pose.orientation.w;


        // RCLCPP_INFO(this->get_logger(), "Received PoseStamped message:\n Position: (%f, %f, %f)\n Orientation: (%f, %f, %f, %f)",
        //             *quad_pose[0], *quad_pose[1],*quad_pose[1],
        //             msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
 
 

    }




    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_pose[0]= msg->pose.position.x;
        goal_pose[1]= msg->pose.position.y;
        goal_pose[2]= msg->pose.position.z;
        goal_pose[3]= msg->pose.orientation.x;
        goal_pose[4]= msg->pose.orientation.y;
        goal_pose[5]= msg->pose.orientation.z;
        goal_pose[6]= msg->pose.orientation.w;

    } 



    void timer_callback()
    {
       auto plannedPath = nav_msgs::msg::Path();
        // RCLCPP_INFO(this->get_logger(), "Publishing: ");
        // Check is service is available
      //   while (!client_->wait_for_service(1s)) {
      //   // If ROS is shutdown before the service is activated, show this error
      //     if (!rclcpp::ok()) {
      //       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      //       rclcpp::shutdown();

      //     }
      //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      // }

        ompl::geometric::SimpleSetup ss_p = planner_.configure(quad_pose,goal_pose);
        ompl::geometric::PathGeometric path1 = planner_.planPath(ss_p);
        ompl::geometric::PathGeometric* path = &path1;

        nav_msgs::msg::Path xd = planner_.extractPath(path);

        publisher_->publish(xd);
        std::cout << "Path published.\n";
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;


    
};

void thread_publisher(void){

    rclcpp::spin(std::make_shared<PathPublisher>());

}









int main(int argc, char * argv[]){
    // create ompl planner
    rclcpp::init(argc, argv);



    std::thread thread_pub = std::thread(thread_publisher);


    thread_pub.join();

    rclcpp::shutdown();
    return 0;
}