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
  double resolution = 0.2;
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


bool checkRayBoxIntersection(btDynamicsWorld* dynamicsWorld, const btVector3& P1, const btVector3& P2, const std::string& Start ,const std::string& End )
{
    btCollisionWorld::ClosestRayResultCallback rayCallback(P1, P2);
    dynamicsWorld->rayTest(P1, P2, rayCallback);

    if (rayCallback.hasHit()) {
        // std::cout << "No line of sight between " + Start + " and " + End + "!" << std::endl;
        return true;
    } else {
        // std::cout << "Line of sight between " + Start + " and " + End + "!" << std::endl;
        return false;
    }
}
bool checkRayBoxIntersection(const moveit::core::RobotState& robot_state,btDynamicsWorld* dynamicsWorld)
{
    Eigen::Isometry3d drone1_transform = robot_state.getFrameTransform("Drone1");
    Eigen::Isometry3d drone2_transform = robot_state.getFrameTransform("Drone2");
    Eigen::Isometry3d drone3_transform = robot_state.getFrameTransform("Drone3");
    Eigen::Isometry3d drone4_transform = robot_state.getFrameTransform("Drone4");
    Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();     

    btVector3 P1(drone1_transform.translation().x(), drone1_transform.translation().y(), drone1_transform.translation().z());  // Start point of the line
    btVector3 P2(drone2_transform.translation().x(), drone2_transform.translation().y(), drone2_transform.translation().z());
    btVector3 P3(drone3_transform.translation().x(), drone3_transform.translation().y(), drone3_transform.translation().z()); 
    btVector3 P4(drone4_transform.translation().x(), drone4_transform.translation().y(), drone4_transform.translation().z());  // Start point of the line
    btVector3 Origin(origin.translation().x(), origin.translation().y(), origin.translation().z()); 
    // End point of the line
    bool intersects1 = checkRayBoxIntersection(dynamicsWorld, P3, P4,"Drone3","Drone4");
    bool intersects2 = checkRayBoxIntersection(dynamicsWorld, P2, P3,"Drone2","Drone3");
    bool intersects3 = checkRayBoxIntersection(dynamicsWorld, P1, P2,"Drone1","Drone2");
    bool intersects4 = checkRayBoxIntersection(dynamicsWorld, Origin, P1,"Origin","Drone1");
    if (intersects1 || intersects2 || intersects3 || intersects4 )
    {
      return false;
    }
    return true;
    }


btDiscreteDynamicsWorld* configure_bullet(std::shared_ptr<planning_scene::PlanningScene> planning_scene)
{

  btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
  btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
  btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();
  btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
  btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);    // Create collision objects
  // const Eigen::Affine3d object_transform = planning_scene->getFrameTransform("box3");
  const collision_detection::WorldConstPtr& world = planning_scene->getWorld();
  std::vector< std::string > object_ids = world->getObjectIds();
  for (size_t i = 0; i < object_ids.size(); ++i)
  {
      const collision_detection::World::ObjectConstPtr& object = world->getObject(object_ids[i]);
    //  object->shapes_[0]
      auto shape = object->shapes_[0]->type;
      Eigen::Affine3d object_pose = planning_scene->getFrameTransform(object_ids[i]);
      Eigen::Quaterniond quaternion(object_pose.rotation());
      // The object in the scene only have one box shape. Should check if more shape types are added. 
      const shapes::Box* box_shape = dynamic_cast<const shapes::Box*>( object->shapes_[0].get());
      btCollisionShape* boxShape = new btBoxShape(btVector3(box_shape->size[0]/2, box_shape->size[1]/2, box_shape->size[2]/2));  // Box size (1x1x1)
      btDefaultMotionState* boxMotionState = new btDefaultMotionState(btTransform(btQuaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()),btVector3(object_pose.translation().x(),object_pose.translation().y(),object_pose.translation().z()))); // Position at origin


      btRigidBody::btRigidBodyConstructionInfo boxRigidBodyCI(0, boxMotionState, boxShape, btVector3(0, 0, 0));
      btRigidBody* boxRigidBody = new btRigidBody(boxRigidBodyCI);
      dynamicsWorld->addRigidBody(boxRigidBody);
  }




  return dynamicsWorld;


  // RCLCPP_INFO(node->get_logger(),"The value of my_bool is: %s", intersects ? "true" : "false");

}


std::vector<std::vector<double>> calculate_cartesian_distances_with_average_matrix(
    const trajectory_msgs::msg::JointTrajectory &trajectory,  // Trajectory input
    const std::vector<std::string> &link_names,               // List of target links
    const moveit::core::RobotModelPtr &robot_model,           // Robot model for FK
    rclcpp::Logger logger                                     // ROS2 logger
) {
    // Check if trajectory has enough points
    if (trajectory.points.size() < 2) {
        RCLCPP_WARN(logger, "Trajectory must have at least 2 points to calculate Cartesian distances.");
        return {};
    }

    // Create a RobotState to perform FK
    moveit::core::RobotState robot_state(robot_model);
    robot_state.setToDefaultValues();

    // Initialize a vector to store total distances for each link
    std::vector<double> total_distances(link_names.size(), 0.0);

    // Map to store the previous pose for each link
    std::vector<Eigen::Isometry3d> prev_poses(link_names.size());

    // Loop through the trajectory points
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
        // Set joint values from the trajectory
        robot_state.setVariablePositions(trajectory.points[i].positions);

        for (size_t link_idx = 0; link_idx < link_names.size(); ++link_idx) {
            // Get the current pose of the link
            const Eigen::Isometry3d &current_pose = robot_state.getGlobalLinkTransform(link_names[link_idx]);

            if (i > 0) {
                // Calculate the Euclidean distance from the previous pose
                Eigen::Vector3d diff = current_pose.translation() - prev_poses[link_idx].translation();
                double distance = diff.norm();

                // Accumulate the distance
                total_distances[link_idx] += distance;
            }

            // Update the previous pose
            prev_poses[link_idx] = current_pose;
        }
    }

    // Calculate the average distance
    double total_distance_sum = 0.0;
    for (double distance : total_distances) {
        total_distance_sum += distance;
    }
    double average_distance = total_distance_sum / link_names.size();

    // Create a matrix (5 rows: 4 links + 1 average, 1 column for each value)
    std::vector<std::vector<double>> distance_matrix(5, std::vector<double>(1));
    for (size_t i = 0; i < total_distances.size(); ++i) {
        distance_matrix[i][0] = total_distances[i];  // Add link distances
        // RCLCPP_INFO(logger, "Link '%s' traveled a distance of %.3f meters", link_names[i].c_str(), total_distances[i]);
    }
    distance_matrix[4][0] = average_distance;       // Add the average
    // RCLCPP_INFO(logger, "Average distance traveled by all links: %.3f meters", average_distance);

    return distance_matrix;
}
















int main(int argc, char * argv[])
{  
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("cpp_demo", node_options);  
  rclcpp::executors::SingleThreadedExecutor executor;
  auto start_time = node->now();
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  // rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr trajectory_publisher_;
   rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
  marker_array_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("viz_const", 10);
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::MarkerArray marker_array2;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
  publisher_ = node->create_publisher<nav_msgs::msg::Path>("/cartesian_path_topic", 10);
  // trajectory_publisher_ = node->create_publisher<moveit_msgs::msg::RobotTrajectory>("trajectory_isaac", 10);
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



  std::vector<std::string> link_names = {"Drone1", "Drone2", "Drone3", "Drone4"};

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

  auto dynamicsWorld = configure_bullet(planning_scene);

  moveit::core::GroupStateValidityCallbackFn callback_fn = [planning_scene,dynamicsWorld,&visual_tools](moveit::core::RobotState* robot_state, const moveit::core::JointModelGroup* joint_group, const double* joint_group_variable_values)
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

    // visual_tools.publishRobotState(joint_values, joint_group, rviz_visual_tools::BLUE);
    // visual_tools.trigger();

    // check if IK solution is in collision
    if (planning_scene->isStateColliding(dummy_state,"Group1",true)||!checkRayBoxIntersection(dummy_state,dynamicsWorld)){
    // RCLCPP_INFO(LOGGER, "not valid ik sol");
      return false;
    }

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

  std::vector<std::vector<double>> all_distances_matrix;
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
  waypoints.pose.position.x += 0 ;
  waypoints.pose.position.y += 3;
  waypoints.pose.position.z += 0;
  waypoints_vec.push_back(waypoints);
  waypoints.pose.position.x += 2 ;
  waypoints.pose.position.y += 0;
  waypoints.pose.position.z += 0;
  waypoints_vec.push_back(waypoints);
  waypoints.pose.position.x += 0 ;
  waypoints.pose.position.y += -3;
  waypoints.pose.position.z += 0;
  waypoints_vec.push_back(waypoints);
  waypoints.pose.position.x += 3 ;
  waypoints.pose.position.y += 0;
  waypoints.pose.position.z += 0;
  waypoints_vec.push_back(waypoints);
  double timeout = 2.0;
  
// -------------------------------------------------- Planning ---------------------------------------------------------------
  // compute_path(publisher_,waypoints_vec);
  nav_msgs::msg::Path path = interpolate_path(publisher_,waypoints_vec);
  // Initializing PlanningPipeline
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
    new planning_pipeline::PlanningPipeline(robot_model, node, "ompl"));
  
    moveit::core::RobotState goal_state(robot_model);
    std::vector<double> joint_values1;


    float fail_counter = 0;
    float counter = 0;
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

      std::function<bool( const moveit::core::RobotState&, bool)> feasible_predicate = [node,dynamicsWorld](const moveit::core::RobotState& robot_state,bool){

        bool LoS = checkRayBoxIntersection(robot_state,dynamicsWorld);
        if (LoS)
        {
          return true;
        }
        else
        {
          return false;
        }
        RCLCPP_INFO(LOGGER,"%s", LoS ? "true" : "false");  
      };
      planning_scene->setStateFeasibilityPredicate(feasible_predicate) ;











// ------------------------------------------------------------------------------------------------------------------------------------



// ------------------------------------------------------------------------------------------------------------------------------------





      planning_interface::MotionPlanRequest req;
      req.pipeline_id = "ompl";
      // req.planner_id = "PRMstar";
      req.planner_id = "PRM";
      req.allowed_planning_time = 5.0;
      req.max_velocity_scaling_factor = 0.6;
      req.num_planning_attempts = 20.0;
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
      box.dimensions ={0.1};  // Box dimensions (x, y, z)
        
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
      // req.path_constraints.clear();


    
    
    
      if(goal_state.setFromIK(joint_model_group, ik_pose, timeout,callback_fn)){
        moveit_msgs::msg::Constraints joint_goal =
          kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group, tolerance_above, tolerance_below); 
         // found_ik = goal_state.setFromIK(joint_model_group, ik_pose, timeout,callback_fn);
        RCLCPP_ERROR(LOGGER, "IK valid.");

        req.goal_constraints.push_back(joint_goal);
      }
      counter++;
      req.goal_constraints.push_back(constraints);
      while (((!planning_pipeline->generatePlan(planning_scene, req, res) || res.error_code_.val != res.error_code_.SUCCESS)) && rclcpp::ok())
      {
        RCLCPP_INFO(LOGGER, "Failed to solve planning problem. Retrying");
        fail_counter++;
        counter++;
      }
      if(planning_scene->isPathValid(*res.trajectory_)){
      moveit_msgs::msg::MotionPlanResponse msg;
      RCLCPP_INFO(LOGGER, "Planning took %.2f seconds", res.planning_time_);
      visual_tools.publishTrajectoryLine(res.trajectory_,joint_model_group);
      visual_tools.trigger();
      RCLCPP_INFO(LOGGER, "Plan is valid");
      res.getMessage(msg);
      if (move_group_interface.execute(msg.trajectory) == true) {
          
          auto distances = calculate_cartesian_distances_with_average_matrix(msg.trajectory.joint_trajectory,link_names,robot_model,LOGGER);
          all_distances_matrix.push_back(distances[4]);

          RCLCPP_INFO(LOGGER, "Plan executed successfully!");
          // trajectory_publisher_->publish(msg.trajectory);
      }
      }
      else
      {
        rclcpp::shutdown();  

      }
          
     
    }

	


  // Shutdown ROS 
  auto end_time = node->now();
  rclcpp::Duration execution_duration = end_time - start_time;
  RCLCPP_INFO(node->get_logger(), "Execution took %f seconds and %ld nanoseconds.",
              execution_duration.seconds(), execution_duration.nanoseconds());
  
      double total_sum = 0.0;
    size_t total_elements = 0;

    for (const auto &row : all_distances_matrix) {
        for (double value : row) {
            total_sum += value;
            total_elements++;
        }
    }
  double overall_average = total_elements > 0 ? (total_sum / total_elements) : 0.0;
  RCLCPP_INFO(node->get_logger(), "Overall Average Distance: %.3f meters", overall_average);
  RCLCPP_INFO(node->get_logger(), "Overall  Distance: %.3f meters", total_sum);

  float rate = (counter-fail_counter)/counter;
  RCLCPP_INFO(node->get_logger(), "Float1: %.2f, Float2: %.3f,Float2: %.3f", counter, fail_counter,rate);
  rclcpp::shutdown();  
  return 0;
}
