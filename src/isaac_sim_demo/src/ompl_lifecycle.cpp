// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcutils/logging_macros.h"

#include "std_msgs/msg/string.hpp"
#include "isaac_sim_demo/RRT_controller.hpp"
#include <Eigen/Dense>
#include "std_srvs/srv/trigger.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp> 

using namespace std::chrono_literals;


class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// LifecycleTalker constructor
  explicit LifecycleTalker(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {
    RCLCPP_INFO(get_logger(), "Node is created");
  }




//   void goal_callback(std_msgs::msg::String::ConstSharedPtr msg)
//   {
//     RCLCPP_INFO(get_logger(), "data_callback: %s", msg->data.c_str());
//   }









  /// Callback for walltimer in order to publish the message.
  /**
   * Callback for walltimer. This function gets invoked by the timer
   * and executes the publishing.
   * For this demo, we ask the node for its current state. If the
   * lifecycle publisher is not activate, we still invoke publish, but
   * the communication is blocked so that no messages is actually transferred.
   */
  void publish()
  {


    auto plannedPath = nav_msgs::msg::Path();
    
    // // Print the current state for demo purposes
    // if (!publisher_->is_activated()) {
    //   RCLCPP_INFO(
    //     get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.");
    // } else {
    //   RCLCPP_INFO(
    //     get_logger(), "Lifecycle publisher is active. Publishing");
    // }
    ompl::geometric::SimpleSetup ss_p = planner_.configure(quad_pose,goal_pose);
    ompl::geometric::PathGeometric path1 = planner_.planPath(ss_p);
    ompl::geometric::PathGeometric* path = &path1;

    nav_msgs::msg::Path xd = planner_.extractPath(path);
    publisher_->publish(xd);
    std::cout << "Path published.\n";
    // We independently from the current state call publish on the lifecycle
    // publisher.
    // Only if the publisher is in an active state, the message transfer is
    // enabled and the message actually published.
    // pub_->publish(std::move(msg));
  }

  /// Transition callback for state configuring
  /**
   * on_configure callback is being called when the lifecycle node
   * enters the "configuring" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "inactive" state or stays
   * in "unconfigured".
   * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
   * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    // This callback is supposed to be used for initialization and
    // configuring purposes.
    // We thus initialize and configure our publishers and timers.
    // The lifecycle node API does return lifecycle components such as
    // lifecycle publishers. These entities obey the lifecycle and
    // can comply to the current state of the node.
    // As of the beta version, there is only a lifecycle publisher
    // available.


    this->declare_parameter("Quadrotor", "Quadrotor_0");
        
        // Create workspace service client
        // client_ = this->create_client<custom_interfaces::srv::PlannerWorkspace>("ws_marker_pub");
        
        // get parameters
    std::string Quadrotor_num = this->get_parameter("Quadrotor").as_string();
    RCLCPP_INFO(get_logger(), "%s.",Quadrotor_num.c_str());

    publisher_ = this->create_publisher<nav_msgs::msg::Path>("/"+Quadrotor_num+"/path_topic", 10);
        
    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        ""+Quadrotor_num+"/state/pose", 10, std::bind(&LifecycleTalker::pose_callback, this, std::placeholders::_1));

    goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/"+Quadrotor_num+"/goal_pose", 10, std::bind(&LifecycleTalker::goal_callback, this, std::placeholders::_1));
        
    // init timer
    timer_ = this->create_wall_timer(
        800ms, std::bind(&LifecycleTalker::publish,this));

    RCLCPP_INFO(get_logger(), "on_configure() is called.");

    // We return a success and hence invoke the transition to the next
    // step: "inactive".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "unconfigured" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /// Transition callback for state activating
  /**
   * on_activate callback is being called when the lifecycle node
   * enters the "activating" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "active" state or stays
   * in "inactive".
   * TRANSITION_CALLBACK_SUCCESS transitions to "active"
   * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state)
  {
    // The parent class method automatically transition on managed entities
    // (currently, LifecyclePublisher).
    // pub_->on_activate() could also be called manually here.
    // Overriding this method is optional, a lot of times the default is enough.
    LifecycleNode::on_activate(state);

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

    // Let's sleep for 2 seconds.
    // We emulate we are doing important
    // work in the activating phase.
    // std::this_thread::sleep_for(2s);

    // We return a success and hence invoke the transition to the next
    // step: "active".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "inactive" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /// Transition callback for state deactivating
  /**
   * on_deactivate callback is being called when the lifecycle node
   * enters the "deactivating" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "inactive" state or stays
   * in "active".
   * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
   * TRANSITION_CALLBACK_FAILURE transitions to "active"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state)
  {
    // The parent class method automatically transition on managed entities
    // (currently, LifecyclePublisher).
    // pub_->on_deactivate() could also be called manually here.
    // Overriding this method is optional, a lot of times the default is enough.
    LifecycleNode::on_deactivate(state);

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

    // We return a success and hence invoke the transition to the next
    // step: "inactive".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "active" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /// Transition callback for state cleaningup
  /**
   * on_cleanup callback is being called when the lifecycle node
   * enters the "cleaningup" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "unconfigured" state or stays
   * in "inactive".
   * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
   * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    // In our cleanup phase, we release the shared pointers to the
    // timer and publisher. These entities are no longer available
    // and our node is "clean".
    timer_.reset();
    publisher_.reset();
    pose_subscriber_.reset();
    goal_subscriber_.reset();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

    // We return a success and hence invoke the transition to the next
    // step: "unconfigured".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "inactive" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /// Transition callback for state shutting down
  /**
   * on_shutdown callback is being called when the lifecycle node
   * enters the "shuttingdown" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "finalized" state or stays
   * in its current state.
   * TRANSITION_CALLBACK_SUCCESS transitions to "finalized"
   * TRANSITION_CALLBACK_FAILURE transitions to current state
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state)
  {
    // In our shutdown phase, we release the shared pointers to the
    // timer and publisher. These entities are no longer available
    // and our node is "clean".
    timer_.reset();
    publisher_.reset();
    pose_subscriber_.reset();
    goal_subscriber_.reset();

    RCUTILS_LOG_INFO_NAMED(
      get_name(),
      "on shutdown is called from state %s.",
      state.label().c_str());

    // We return a success and hence invoke the transition to the next
    // step: "finalized".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the current state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
    // We hold an instance of a lifecycle publisher. This lifecycle publisher
    // can be activated or deactivated regarding on which state the lifecycle node
    // is in.
    // By default, a lifecycle publisher is inactive by creation and has to be
    // activated to publish messages into the ROS world.

    // We hold an instance of a timer which periodically triggers the publish function.
    // As for the beta version, this is a regular timer. In a future version, a
    // lifecycle timer will be created which obeys the same lifecycle management as the
    // lifecycle publisher.
    std::vector<double> quad_pose = std::vector<double>(7);
    std::vector<double> goal_pose = std::vector<double>(7);

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> publisher_;
    // rclcpp::<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
    // planner 
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

        RCLCPP_INFO(this->get_logger(), "Received PoseStamped message:\n Position: (%f, %f, %f)\n Orientation: (%f, %f, %f, %f)",
                        quad_pose[0], quad_pose[1],quad_pose[1],
                        msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
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

};

/**
 * A lifecycle node has the same node API
 * as a regular node. This means we can spawn a
 * node, give it a name and add it to the executor.
 */
int main(int argc, char * argv[])
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<LifecycleTalker> lc_node =
    std::make_shared<LifecycleTalker>("path_publisher");

  exe.add_node(lc_node->get_node_base_interface());

  
  // Transition to inactive state

  RCLCPP_INFO(lc_node->get_logger(), "Configuring node...");
  lc_node->configure();
  exe.spin_some();

  // Transition to activate state
  RCLCPP_INFO(lc_node->get_logger(), "Activating node...");
  lc_node->activate();
  exe.spin_some();


  exe.spin();







  rclcpp::shutdown();

  return 0;
}