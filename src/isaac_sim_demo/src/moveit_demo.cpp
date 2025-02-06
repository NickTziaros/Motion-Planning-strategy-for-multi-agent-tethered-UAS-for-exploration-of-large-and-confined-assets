#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#pragma GCC diagnostic ignored "-Wunused-variable"

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "moveit_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_demo");

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });





  // Next step goes here
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "Group1");
  move_group_interface.allowLooking(true);
  static const std::string PLANNING_GROUP = "Group1";
  namespace rvt = rviz_visual_tools;
  std::string planner_id = "RRTConnect";  // Example: RRTConnect
  // move_group_interface.setPlannerId(planner_id);
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link","display_contacts",
                                                      move_group_interface.getRobotModel());
  move_group_interface.setPlannerId(planner_id);
  // move_group_interface.setEndEffector("Drone4");
    // Print the planning pipeline and planner ID being used
  std::string planning_pipeline = move_group_interface.getPlannerId();
  RCLCPP_INFO(logger, "Using : %s", planning_pipeline.c_str());
  move_group_interface.setPlanningTime(30);
  // move_group_interface.setGoalOrientationTolerance(1.5707)  ;

  // RCLCPP_INFO(logger, "getDefaultPlannerId : %s", 	move_group_interface.getDefaultPlannerId().c_str());
  RCLCPP_INFO(logger, "getVariableCount : %d", 	move_group_interface.getVariableCount());
  // move_group_interface.setWorkspace()
  // RCLCPP_INFO(logger, "Using planner: %s", planner_id.c_str());

  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // auto objects = planning_scene_interface.getObjects();
  // RCLCPP_INFO(logger, "The planning scene contains %zu objects.", objects.size());

  // RCLCPP_INFO(logger, "Using planner: %s", planning_scene_interface.getObjects());

  // ------------------------------------------------------------------------------------------------

  // ------------------------------------------------------------------------------------------------


    // move_group_interface.setPoseTarget(target_pose);

    // Add the collision object to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;




























// -----------------------------------------------------------------------------------------------

  // Set a target Pose
  moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
  const moveit::core::JointModelGroup* joint_model_group = start_state.getJointModelGroup("Group1");

    // Get the names of the joints in the group
  // const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    // Get the joint positions
  std::vector<double> joint_values;
  start_state.copyJointGroupPositions(joint_model_group, joint_values);
    
    // Print the joint names and their values
  // for (size_t i = 0; i < joint_names.size(); ++i)
  //   {
  //       RCLCPP_INFO(node->get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  //   }

  const Eigen::Isometry3d& end_effector_state = start_state.getGlobalLinkTransform("Drone4");
  // RCLCPP_INFO_STREAM(node->get_logger(), "Translation: \n" << end_effector_state.translation() << "\n");
  // RCLCPP_INFO_STREAM(node->get_logger(), "Rotation: \n" << end_effector_state.rotation() << "\n");
  double timeout = 1;
  // bool found_ik = robot_state->setFromIK(joint_model_group, new_transform, timeout);
  // if (found_ik)
  // {
  //   robot_state->copyJointGroupPositions(joint_model_group, joint_values);
  //   for (std::size_t i = 0; i < joint_names.size(); ++i)
  //   {
  //     RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  //   }
  
  // }
  geometry_msgs::msg::Pose ee_pose;
  ee_pose.orientation.w = 1.0;
  ee_pose.position.x = end_effector_state.translation().x();
  ee_pose.position.y = end_effector_state.translation().y();
  ee_pose.position.z = end_effector_state.translation().z();


  std::vector<geometry_msgs::msg::Pose> waypoints_vec;

  geometry_msgs::msg::Pose waypoints;
  // waypoints.orientation.w = 1.00000;
  // waypoints.orientation.x = 1e-6;
  // waypoints.orientation.y = 1e-6;
  // waypoints.orientation.z = 1e-6;
  waypoints.position.x = end_effector_state.translation().x();
  waypoints.position.y = end_effector_state.translation().y()+1.5;
  waypoints.position.z = end_effector_state.translation().z()+0.3;
  // waypoints_vec.push_back(waypoints);

  // waypoints.orientation.w = 1.0;
  // waypoints.position.z += 0.25;
  // waypoints_vec.push_back(waypoints);
  // waypoints.position.y += 0.5;
  // waypoints_vec.push_back(waypoints);
  // waypoints.position.y += 0.5;
  // waypoints_vec.push_back(waypoints);
  // waypoints.position.y += 0.5;
  // waypoints_vec.push_back(waypoints);
  // waypoints.position.y += 0.5;
  // waypoints_vec.push_back(waypoints);
  // waypoints.position.y += 0.5;
  // waypoints_vec.push_back(waypoints);
  // waypoints.position.y += 0.5;
  // waypoints_vec.push_back(waypoints);
  // waypoints.position.y += 0.5;
  // waypoints_vec.push_back(waypoints);
  // for (std::size_t i = 0; i < waypoints_vec.size(); ++i)
  // {
  //   visual_tools.publishAxisLabeled(waypoints_vec[i], "pt" + std::to_string(i), rvt::XXLARGE);
  //   RCLCPP_INFO_STREAM(node->get_logger(), "Point: \n" << waypoints_vec[i].position.y << "\n");
  // }
  
  moveit::core::RobotState new_state(*move_group_interface.getCurrentState());
  std::vector<std::vector<double>> ik_solutions;

  while(ik_solutions.size() < 10 && rclcpp::ok())
  {
    bool found_ik = new_state.setFromIK(joint_model_group, waypoints, timeout);
    if (found_ik)
    {
      RCLCPP_INFO(node->get_logger(), "Found ");

      new_state.copyJointGroupPositions(joint_model_group, joint_values);
      const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
      ik_solutions.push_back(joint_values);
      move_group_interface.setJointValueTarget(joint_values);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    }
    else
    {
      
      RCLCPP_INFO(node->get_logger(), "IK for waypoint not found. Retrying....");

    }





  }

  for (std::size_t i = 0; i < ik_solutions.size(); ++i)
    {    
          if (i > 0) {
        // Compare current solution with the previous one
        if (ik_solutions[i] != ik_solutions[i - 1]) 
        {
            RCLCPP_INFO(node->get_logger(), "diff solution"); 
        }
    }
      visual_tools.publishRobotState(ik_solutions[i],joint_model_group,rviz_visual_tools::BLUE);
      visual_tools.trigger();
      RCLCPP_INFO(node->get_logger(), "Publishing IK solution");

      rclcpp::sleep_for(std::chrono::seconds(2));
    }


  // for (std::size_t i = 0; i < waypoints_vec.size(); ++i)
  // {

  //   // move_group_interface.setPoseTarget(waypoints_vec[i]);


    // moveit::core::RobotState new_state(*move_group_interface.getCurrentState());
  //   const moveit::core::JointModelGroup* joint_model_group = new_state.getJointModelGroup("Group1");

  //   double timeout = 0.5;
  //   bool found_ik = false;
  //   new_state.printStatePositions();

  //   while(found_ik == false && rclcpp::ok())
  //   {
  //   found_ik = new_state.setFromIK(joint_model_group, waypoints_vec[i], timeout);
  //   RCLCPP_ERROR(node->get_logger(), "IK for waypoint not found. Retrying....");

  //   }
  //   if (found_ik)
  //     {
  //         RCLCPP_INFO(node->get_logger(), "IK for waypoint found");
  //         new_state.copyJointGroupPositions(joint_model_group, joint_values);

  //         move_group_interface.setJointValueTarget(joint_values);

  //         moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //         // bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  
  
  //       //   if (success)
  //       // {
  //       //   RCLCPP_WARN(node->get_logger(), "Success ");
  //       //   // move_group_interface.move();
  //       //   visual_tools.deleteAllMarkers();
  //       //   // visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XXLARGE);
  //       //   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  //       //   visual_tools.trigger();

  //       // }

  //     }


  // {
    
  //   for (std::size_t i = 0; i < joint_names.size(); ++i)
  //   {
  //     RCLCPP_INFO(node->get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  //   }
  // }
  // else
  // {
  //   RCLCPP_INFO(node->get_logger(), "Did not find IK solution");
  // }



  

//  }
    // auto const [success, plan] = [&move_group_interface] {
    //   moveit::planning_interface::MoveGroupInterface::Plan msg;
    //   auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    //   return std::make_pair(ok, msg);
    // }();

    // // Execute the plan
    // if (success) {
    
    //   RCLCPP_ERROR(logger, "Planing Succeded!");

    //   move_group_interface.move();
    // } else {
    //   RCLCPP_ERROR(logger, "Planing failed!");
    // }
  



      // Shutdown ROS 
    rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
    spinner.join();  // <--- Join the thread before exiting
    return 0;
}