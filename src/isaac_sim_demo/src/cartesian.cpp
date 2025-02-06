#include <pluginlib/class_loader.hpp>
#include <math.h>

// MoveIt
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/robot_state/conversions.h>  
#include <moveit/move_group_interface/move_group_interface.h>
#include <nav_msgs/msg/path.hpp> 
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-parameter"
static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning_pipeline");
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit_msgs/msg/visibility_constraint.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <fcl/fcl.h>
#include <btBulletDynamicsCommon.h>

void compute_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& publisher_,std::vector<geometry_msgs::msg::PoseStamped>& waypoints){

  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  for (size_t i = 0; i < waypoints.size(); i++)
  {
    path.poses.push_back(waypoints[i]);
  }
  publisher_->publish(path);
}





nav_msgs::msg::Path interpolate_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& publisher_,std::vector<geometry_msgs::msg::PoseStamped>& waypoints){
  std::vector<geometry_msgs::msg::PoseStamped> interpolated_points;
  double resolution = 0.5;
  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  for (size_t i = 0; i < waypoints.size()-1; i++)
  {
    geometry_msgs::msg::PoseStamped start = waypoints[i];
    geometry_msgs::msg::PoseStamped end = waypoints[i + 1];

    double dx = end.pose.position.x - start.pose.position.x;
    double dy = end.pose.position.y - start.pose.position.y;
    double dz = end.pose.position.z - start.pose.position.z;

    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    int steps = std::max(1, static_cast<int>(distance / resolution));
    
    for (int j = 0; j < steps; ++j) {
        double t = static_cast<double>(j) / steps;
        geometry_msgs::msg::PoseStamped interpolated;
        interpolated.pose.position.x = start.pose.position.x + t * dx;
        interpolated.pose.position.y = start.pose.position.y + t * dy;
        interpolated.pose.position.z= start.pose.position.z + t * dz;
        
        interpolated_points.push_back(interpolated);
        path.poses.push_back(interpolated);
    }



    // path.poses.push_back(waypoints[end]);
  }
  publisher_->publish(path);
  return(path);
}









int main(int argc, char * argv[])
{  
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("cpp_demo", node_options);  
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
  marker_array_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("viz_const", 10);
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::MarkerArray marker_array2;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
  publisher_ = node->create_publisher<nav_msgs::msg::Path>("/cartesian_path_topic", 10);
  namespace rvt = rviz_visual_tools;
  // rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher =
  //     node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
// --------------------------------------- Robot_State && PlanningScene -----------------------------------------------
  std::vector<double> joint_values;
  const std::string PLANNING_GROUP = "Group1";
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader(node, "robot_description"));

  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr client_;
  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();
  moveit::core::RobotState robot_state(robot_model);
  
  // RVT
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link", "display_contacts", robot_model);
  
  // Init move_group for trajectory execution 
  moveit::planning_interface::MoveGroupInterface move_group_interface(node, PLANNING_GROUP);

  // Creating PlanningScene object
  std::shared_ptr<planning_scene::PlanningScene> planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);




  bool found_ik = false;
  bool isValid = false;
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  
// --------------------------------------- Planning_Scene_Srv -----------------------------------------------
  client_ = node->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
  while (!client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
      RCLCPP_INFO(LOGGER, "Waiting for GetPlanningScene service...");
  }

  auto future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS){
    auto response = future.get();  // Access the service response
    RCLCPP_INFO(node->get_logger(), "Received planning scene response");
    planning_scene->usePlanningSceneMsg(response->scene);
    robot_state = planning_scene->getCurrentState();

    // Example: print the number of world objects
    size_t num_world_objects = response->scene.world.collision_objects.size();
    RCLCPP_INFO(node->get_logger(), "Number of collision objects in the scene: %ld", num_world_objects);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service get_planning_scene");
  }

  moveit::core::GroupStateValidityCallbackFn callback_fn = [planning_scene,&visual_tools](moveit::core::RobotState* robot_state, const moveit::core::JointModelGroup* joint_group, const double* joint_group_variable_values)
  {
    // Check for robot state
    if (!robot_state) {
        RCLCPP_ERROR(LOGGER, "RobotState is null.");
        return false;
    }

    // Check for planning scene
    if (!planning_scene) {
        RCLCPP_ERROR(LOGGER, "Planning scene is null.");
        return false;
    }

    // visualise IK solution
    moveit::core::RobotState dummy_state = planning_scene->getCurrentState();
    std::vector<double> joint_values(joint_group_variable_values, 
                                     joint_group_variable_values + joint_group->getVariableCount());
    dummy_state.setJointGroupPositions(joint_group, joint_values);

    visual_tools.publishRobotState(joint_values, joint_group, rviz_visual_tools::BLUE);
    visual_tools.trigger();
    // check if IK solution is in collision
    // if (planning_scene->isStateColliding(dummy_state,"Group1",true)){
    // RCLCPP_INFO(LOGGER, "not valid ik sol");
    //   return false;
    // }


    return true;

  };




// --------------------------------------------------- Path -------------------------------------------------------
  const moveit::core::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("Group1");
  const Eigen::Isometry3d& end_effector_state = robot_state.getGlobalLinkTransform("Drone4");
  // RCLCPP_INFO(LOGGER, "Translation: [%f, %f, %f]",end_effector_state.translation().x(), 
  //             end_effector_state.translation().y(), end_effector_state.translation().z());
  
  geometry_msgs::msg::Pose ee_pose;
  ee_pose.orientation.w = 0;
  ee_pose.orientation.x = 0;
  ee_pose.orientation.y = 0;
  ee_pose.orientation.z = 1.0;
  ee_pose.position.x = end_effector_state.translation().x();
  ee_pose.position.y = end_effector_state.translation().y();
  ee_pose.position.z = end_effector_state.translation().z();


  std::vector<geometry_msgs::msg::PoseStamped> waypoints_vec;

  geometry_msgs::msg::PoseStamped waypoints;

  waypoints.header.frame_id = "base_link";
  waypoints.pose.position.x = end_effector_state.translation().x();
  waypoints.pose.position.y = end_effector_state.translation().y();
  waypoints.pose.position.z = end_effector_state.translation().z();
  waypoints.pose.orientation.w = 1.0;
  waypoints.pose.orientation.y = 1e-6;
  waypoints.pose.orientation.x = 1e-6;
  waypoints.pose.orientation.z = 1e-6;



  // std::vector<geometry_msgs::msg::Pose> waypoints_vec;

  // geometry_msgs::msg::Pose waypoints;
  // waypoints.position.x = end_effector_state.translation().x();
  // waypoints.position.y = end_effector_state.translation().y() + 0.0;
  // waypoints.position.z = end_effector_state.translation().z() + 0.5;
  waypoints_vec.push_back(waypoints);

  waypoints.pose.position.x += 1.0 ;
  waypoints.pose.position.y += 0;
  waypoints.pose.position.z += 0.5;
  waypoints_vec.push_back(waypoints);


  waypoints.pose.position.x += 2 ;
  waypoints.pose.position.y += 0;
  waypoints.pose.position.z += 0;
  waypoints_vec.push_back(waypoints);
  // waypoints.pose.position.x += 0 ;
  // waypoints.pose.position.y += 3;
  // waypoints.pose.position.z += 0;
  // waypoints_vec.push_back(waypoints);
  // waypoints.pose.position.x += 2 ;
  // waypoints.pose.position.y += 0;
  // waypoints.pose.position.z += 0;
  // waypoints_vec.push_back(waypoints);
  // waypoints.pose.position.x += 0 ;
  // waypoints.pose.position.y += -3;
  // waypoints.pose.position.z += 0;
  // waypoints_vec.push_back(waypoints);
  waypoints.pose.position.x += 3 ;
  waypoints.pose.position.y += 0;
  waypoints.pose.position.z += 0;
  waypoints_vec.push_back(waypoints);
  // waypoints.pose.position.x -= 0.5 ;
  // waypoints.pose.position.y += 0;
  // waypoints.pose.position.z += 0;
  // waypoints_vec.push_back(waypoints);   
  double timeout = 2.0;
  
// -------------------------------------------------- Planning ---------------------------------------------------------------
  // compute_path(publisher_,waypoints_vec);
  nav_msgs::msg::Path path = interpolate_path(publisher_,waypoints_vec);
  // Initializing PlanningPipeline
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
    new planning_pipeline::PlanningPipeline(robot_model, node, "ompl"));
  
    moveit::core::RobotState goal_state(robot_model);
    std::vector<double> joint_values1;
  // constraint_samplers::ConstraintSamplerManager sampler_manager;


  // const kinematics::KinematicsBaseConstPtr& solver = joint_model_group->getSolverInstance();
  //   if (!solver)
  //   {
  //     RCLCPP_ERROR(node->get_logger(), "No kinematics solver instantiated for group '%s'", joint_model_group->getName().c_str());
  //     return false;
  //   }
  //   else
  //   {
  //     RCLCPP_ERROR(node->get_logger(), "Kinematics solver instantiated for group '%s'", joint_model_group->getName().c_str());

  //   }


    for (std::size_t i = 1; i < path.poses.size(); ++i){

      geometry_msgs::msg::Pose ik_pose;
      geometry_msgs::msg::PoseStamped ik_pose_stamped;
      ik_pose = path.poses[i].pose; 
      ik_pose_stamped.header.frame_id = "base_link";
      ik_pose_stamped.pose = path.poses[i].pose;

      kinematic_constraints::VisibilityConstraint vis_constraint_checker(robot_model); 
      
      auto future = client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS){
        auto response = future.get();  // Access the service response
        RCLCPP_INFO(node->get_logger(), "Received planning scene response");
        planning_scene->usePlanningSceneMsg(response->scene);
        robot_state = planning_scene->getCurrentState();
        goal_state = planning_scene->getCurrentState();
      } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service get_planning_scene");
      }


      moveit_msgs::msg::VisibilityConstraint visibility_constraint;
      geometry_msgs::msg::PoseStamped sensor_pose;
      sensor_pose.header.frame_id = "Drone3";  // Set the sensor frame to Drone3
      sensor_pose.pose.position.x = 0.0;
      sensor_pose.pose.position.y = 0;
      sensor_pose.pose.position.z = 0.0;
      sensor_pose.pose.orientation.w = 1.0;  // Identity orientation
      visibility_constraint.sensor_pose = sensor_pose;
      visibility_constraint.target_pose.header.frame_id = "Drone4";  // Set target to Drone4
      visibility_constraint.target_pose.pose.position.z=0; 
      visibility_constraint.target_radius = 0.1; 
      visibility_constraint.max_view_angle = 0;  // 180 degrees just for completeness, but not critical here
      visibility_constraint.weight = 1.0;
      visibility_constraint.max_range_angle = 0; 


      Eigen::Isometry3d drone4_transform = robot_state.getFrameTransform("Drone4");

      moveit::core::Transforms transforms("Drone4"); // Provide the target frame
      transforms.setTransform(drone4_transform, "Drone4");
      vis_constraint_checker.configure(visibility_constraint, transforms);
      if (!vis_constraint_checker.configure(visibility_constraint, transforms))
      {
          RCLCPP_ERROR(node->get_logger(), "Failed to configure visibility constraint.");
      }
      else{
          RCLCPP_ERROR(node->get_logger(), "Configured visibility constraint.");

      }
      vis_constraint_checker.getMarkers(robot_state,marker_array);
      marker_array_publisher_->publish(marker_array);
      if(vis_constraint_checker.enabled()){
          RCLCPP_ERROR(node->get_logger(), "Is enabled");
      }
      kinematic_constraints::ConstraintEvaluationResult result = vis_constraint_checker.decide(robot_state,true);
      bool success = result.satisfied;
      if (success){
          RCLCPP_ERROR(node->get_logger(), "SATISFIED------------------------------------");
      }
      else
      {
          RCLCPP_ERROR(node->get_logger(), "NOT SATISFIED");
      }



      



















      
      
      // shapes::Mesh* visibility_cone = vis_constraint_checker.getVisibilityCone(robot_state);
      // Eigen::Affine3d attach_transform = Eigen::Translation3d(0.0, 0.0, 0.5) * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
      // robot_state.attachBody("Cone4",visibility_cone,attach_transform,touch_links,"Drone3")
      moveit_msgs::msg::VisibilityConstraint visibility_constraint2;
      geometry_msgs::msg::PoseStamped sensor_pose2;
      sensor_pose2.header.frame_id = "Drone2";  // Set the sensor frame to Drone3
      sensor_pose2.pose.position.x = 0.0;
      sensor_pose2.pose.position.y = 0.0;
      sensor_pose2.pose.position.z = 0.0;
      sensor_pose2.pose.orientation.w = 1.0;  // Identity orientation
      visibility_constraint2.sensor_pose = sensor_pose2;
      visibility_constraint2.target_pose.header.frame_id = "Drone3";  // Set target to Drone4
      visibility_constraint2.target_radius = 0.2; 
      visibility_constraint2.max_view_angle = 0;  // 180 degrees just for completeness, but not critical here
      visibility_constraint2.weight = 1.0;
      visibility_constraint2.max_range_angle = 0; 
      // while(!planning_scene->isStateValid(goal_state,"Group1") && rclcpp::ok()){
      //   found_ik = goal_state.setFromIK(joint_model_group, ik_pose, timeout,callback_fn);
      //   RCLCPP_ERROR(LOGGER, "Goal State is not valid");
      // }
      Eigen::Isometry3d drone4_transform1 = robot_state.getFrameTransform("Drone3");

      moveit::core::Transforms transforms1("Drone3"); // Provide the target frame
      transforms1.setTransform(drone4_transform1, "Drone3");
      vis_constraint_checker.configure(visibility_constraint2, transforms1);
      if (!vis_constraint_checker.configure(visibility_constraint2, transforms1))
      {
          RCLCPP_ERROR(node->get_logger(), "Failed to configure visibility constraint.");
      }
      else{
          RCLCPP_ERROR(node->get_logger(), "Configured visibility constraint.");

      }
      vis_constraint_checker.getMarkers(robot_state,marker_array2);
      // marker_array_publisher_->publish(marker_array2);
      if(vis_constraint_checker.enabled()){
          RCLCPP_ERROR(node->get_logger(), "Is enabled");
      }
      kinematic_constraints::ConstraintEvaluationResult result1 = vis_constraint_checker.decide(robot_state,true);
      bool success1 = result1.satisfied;
      if (success1){
          RCLCPP_ERROR(node->get_logger(), "SATISFIED------------------------------------");
      }
      else
      {
          RCLCPP_ERROR(node->get_logger(), "NOT SATISFIED");
      }







      planning_interface::MotionPlanRequest req;
      req.pipeline_id = "ompl";
      req.planner_id = "RRTConnect";
      // req.planner_id = "PRM";
      req.allowed_planning_time = 20.0;
      req.max_velocity_scaling_factor = 1.0;
      req.num_planning_attempts = 10.0;
      req.group_name = "Group1";
      req.max_acceleration_scaling_factor = 1.0;
      req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = req.workspace_parameters.min_corner.z = -15.0;
      req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = req.workspace_parameters.max_corner.z = 15.0;
      double tolerance_below = 0.01;
      double tolerance_above = 0.01;
      std::vector<double> tolerance_pose(3, 0.1);
      std::vector<double> tolerance_angle(3, 1);
      planning_interface::MotionPlanResponse res;


      req.goal_constraints.clear();
      
      moveit_msgs::msg::PositionConstraint position_constraint;
      position_constraint.link_name = "Drone4";
      position_constraint.header.frame_id = "base_link";
      shape_msgs::msg::SolidPrimitive box;
      box.type = shape_msgs::msg::SolidPrimitive::SPHERE;
      box.dimensions ={0.2};  // Box dimensions (x, y, z)
        geometry_msgs::msg::PoseStamped box_pose;
      box_pose.header.frame_id = "base_link";  // Reference frame for the constraint
      box_pose.pose.position.x = ik_pose.position.x;  // Box position (x, y, z)
      box_pose.pose.position.y = ik_pose.position.y;
      box_pose.pose.position.z = ik_pose.position.z;
      box_pose.pose.orientation.w = 1.0;  
      position_constraint.constraint_region.primitives.push_back(box);
      position_constraint.constraint_region.primitive_poses.push_back(box_pose.pose);
      position_constraint.weight = 1.0;







      moveit_msgs::msg::Constraints constraints;
      req.goal_constraints.clear();
      constraints.position_constraints.push_back(position_constraint);
      constraints.visibility_constraints.push_back(visibility_constraint);   
      constraints.visibility_constraints.push_back(visibility_constraint2);   
      // req.goal_constraints.decide(robot_state);
    
    
    
      // if(goal_state.setFromIK(joint_model_group, ik_pose, timeout,callback_fn)){
      //   moveit_msgs::msg::Constraints joint_goal =
      //     kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group, tolerance_above, tolerance_below); 
      //    // found_ik = goal_state.setFromIK(joint_model_group, ik_pose, timeout,callback_fn);
      //   RCLCPP_ERROR(LOGGER, "IK valid.");

      //   req.goal_constraints.push_back(joint_goal);

      // }

      req.goal_constraints.push_back(constraints);
      
      // while (((!planning_pipeline->generatePlan(planning_scene, req, res) || res.error_code_.val != res.error_code_.SUCCESS)) && rclcpp::ok())
      // {
      //   RCLCPP_INFO(LOGGER, "Failed to solve planning problem. Retrying");
      // }

      // if(planning_scene->isPathValid(*res.trajectory_)){
      // moveit_msgs::msg::MotionPlanResponse msg;
      // RCLCPP_INFO(LOGGER, "Planning took %.2f seconds", res.planning_time_);
      // visual_tools.publishTrajectoryLine(res.trajectory_,joint_model_group);
      // visual_tools.trigger();
      // RCLCPP_INFO(LOGGER, "Plan is valid");
      // res.getMessage(msg);
      // if (move_group_interface.execute(msg.trajectory) == true) {
      //     RCLCPP_INFO(LOGGER, "Plan executed successfully!");
      // }
      // }
          
        
    }


    // std::cout << "Translation (x, y, z): " 
    //         << drone4_transform.translation().x() << ", " 
    //         << drone4_transform.translation().y() << ", " 
    //         << drone4_transform.translation().z() << std::endl;
    // std::cout << "Translation (x, y, z): " 
    //         << drone3_transform.translation().x() << ", " 
    //         << drone3_transform.translation().y() << ", " 
    //         << drone3_transform.translation().z() << std::endl;
  // Shutdown ROS 
  rclcpp::shutdown();  
  return 0;
}
