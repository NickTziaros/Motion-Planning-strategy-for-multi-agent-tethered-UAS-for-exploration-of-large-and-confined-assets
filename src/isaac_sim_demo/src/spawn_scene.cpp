#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "spaw_scene");

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("spaw_scene");

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   auto spinner = std::thread([&executor]() { executor.spin(); });




// ------------------------------------------------------------------------------------------------
// Create collision object for the robot to avoid
auto const collision_object1 = [frame_id ="base_link"] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box1";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 4;
  primitive.dimensions[primitive.BOX_Y] = 0.5;
  primitive.dimensions[primitive.BOX_Z] = 1;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 4.5;
  box_pose.position.y = 3.0;
  box_pose.position.z = 2.5;
  box_pose.orientation.z = 0.7071;
  box_pose.orientation.w = 0.7071;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}();

auto const collision_object2 = [frame_id ="base_link"] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box2";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 4;
  primitive.dimensions[primitive.BOX_Y] = 0.5;
  primitive.dimensions[primitive.BOX_Z] = 1;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 4.5;
  box_pose.position.y = -3;
  box_pose.position.z = 2.5;
  box_pose.orientation.z = 0.7071;
  box_pose.orientation.w = 0.7071;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}();

auto const collision_object3 = [frame_id ="base_link"] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box3";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 10;
  primitive.dimensions[primitive.BOX_Y] = 0.5;
  primitive.dimensions[primitive.BOX_Z] = 2;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  // box_pose.orientation.w = 1.0;
  box_pose.position.x = 4.5;
  box_pose.position.y = 0;
  box_pose.position.z = 1;
  box_pose.orientation.z = 0.7071;
  box_pose.orientation.w = 0.7071;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}();


auto const collision_object4 = [frame_id ="base_link"] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box4";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 10;
  primitive.dimensions[primitive.BOX_Y] = 0.5;
  primitive.dimensions[primitive.BOX_Z] = 2;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  // box_pose.orientation.w = 1.0;
  box_pose.position.x = 4.5;
  box_pose.position.y = 0;
  box_pose.position.z = 4;
  box_pose.orientation.z = 0.7071;
  box_pose.orientation.w = 0.7071;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}();







auto const collision_object21 = [frame_id ="base_link"] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box21";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 4;
  primitive.dimensions[primitive.BOX_Y] = 0.5;
  primitive.dimensions[primitive.BOX_Z] = 1;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 9.5;
  box_pose.position.y = 3.0;
  box_pose.position.z = 2.5;
  box_pose.orientation.z = 0.7071;
  box_pose.orientation.w = 0.7071;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}();

auto const collision_object22 = [frame_id ="base_link"] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box22";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 4;
  primitive.dimensions[primitive.BOX_Y] = 0.5;
  primitive.dimensions[primitive.BOX_Z] = 1;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 9.5;
  box_pose.position.y = -3;
  box_pose.position.z = 2.5;
  box_pose.orientation.z = 0.7071;
  box_pose.orientation.w = 0.7071;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}();

auto const collision_object23 = [frame_id ="base_link"] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box23";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 10;
  primitive.dimensions[primitive.BOX_Y] = 0.5;
  primitive.dimensions[primitive.BOX_Z] = 2;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 9.5;
  box_pose.position.y = 0;
  box_pose.position.z = 1;
  box_pose.orientation.z = 0.7071;
  box_pose.orientation.w = 0.7071;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}();


auto const collision_object24 = [frame_id ="base_link"] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box24";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 10;
  primitive.dimensions[primitive.BOX_Y] = 0.5;
  primitive.dimensions[primitive.BOX_Z] = 2;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 9.5;
  box_pose.position.y = 0;
  box_pose.position.z = 4;
  box_pose.orientation.z = 0.7071;
  box_pose.orientation.w = 0.7071;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}();











auto const collision_object5 = [frame_id ="base_link"] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box5";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 30;
  primitive.dimensions[primitive.BOX_Y] = 30;
  primitive.dimensions[primitive.BOX_Z] = 0;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0;
  box_pose.position.y = 0;
  box_pose.position.z = -0.005;
  box_pose.orientation.z = 0.7071;
  box_pose.orientation.w = 0.7071;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}();


auto const collision_object6 = [frame_id ="base_link"] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box6";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 11;
  primitive.dimensions[primitive.BOX_Y] = 9.5;
  primitive.dimensions[primitive.BOX_Z] = 0.2;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 9;
  box_pose.position.y = 0;
  box_pose.position.z = 5.10;
  box_pose.orientation.z = 0.7071;
  box_pose.orientation.w = 0.7071;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}();

auto const collision_object7 = [frame_id ="base_link"] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box7";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.3;
  primitive.dimensions[primitive.BOX_Y] = 2;
  primitive.dimensions[primitive.BOX_Z] = 5;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 5.5;
  box_pose.position.y = 2;
  box_pose.position.z = 2.5;
  box_pose.orientation.z = 0.7071;
  box_pose.orientation.w = 0.7071;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}();

auto const collision_object8 = [frame_id ="base_link"] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box8";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.3;
  primitive.dimensions[primitive.BOX_Y] = 2;
  primitive.dimensions[primitive.BOX_Z] = 5;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 9.5;
  box_pose.position.y = 2;
  box_pose.position.z = 2.5;
  box_pose.orientation.z = 0.7071;
  box_pose.orientation.w = 0.7071;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}();


auto const collision_object9 = [frame_id ="base_link"] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box9";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.5;
  primitive.dimensions[primitive.BOX_Y] = 9.5;
  primitive.dimensions[primitive.BOX_Z] = 5;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 9;
  box_pose.position.y = -5.25;
  box_pose.position.z = 2.5;
  box_pose.orientation.z = 0.7071;
  box_pose.orientation.w = 0.7071;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}();

auto const collision_object10 = [frame_id ="base_link"] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box10";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.5;
  primitive.dimensions[primitive.BOX_Y] = 9.5;
  primitive.dimensions[primitive.BOX_Z] = 5;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 9;
  box_pose.position.y = 5.25;
  box_pose.position.z = 2.5;
  box_pose.orientation.z = 0.7071;
  box_pose.orientation.w = 0.7071;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}();














// ------------------------------------------------------------------------------------------------


  // move_group_interface.setPoseTarget(target_pose);

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  
  collision_objects.push_back(collision_object1);
  collision_objects.push_back(collision_object2);
  collision_objects.push_back(collision_object3);
  collision_objects.push_back(collision_object4);
  collision_objects.push_back(collision_object21);
  collision_objects.push_back(collision_object22);
  collision_objects.push_back(collision_object23);
  collision_objects.push_back(collision_object24);
  collision_objects.push_back(collision_object5);
  collision_objects.push_back(collision_object6);
  // collision_objects.push_back(collision_object7);
  // collision_objects.push_back(collision_object8);
  collision_objects.push_back(collision_object9);
  collision_objects.push_back(collision_object10);
  planning_scene_interface.addCollisionObjects(collision_objects);

  planning_scene_interface.applyCollisionObject(collision_object1);
  planning_scene_interface.applyCollisionObject(collision_object2);
  planning_scene_interface.applyCollisionObject(collision_object3);
  planning_scene_interface.applyCollisionObject(collision_object4);
  planning_scene_interface.applyCollisionObject(collision_object21);
  planning_scene_interface.applyCollisionObject(collision_object22);
  planning_scene_interface.applyCollisionObject(collision_object23);
  planning_scene_interface.applyCollisionObject(collision_object24);
  planning_scene_interface.applyCollisionObject(collision_object5);
  planning_scene_interface.applyCollisionObject(collision_object6);
  // planning_scene_interface.applyCollisionObject(collision_object7);
  // planning_scene_interface.applyCollisionObject(collision_object8);
  planning_scene_interface.applyCollisionObject(collision_object9);
  planning_scene_interface.applyCollisionObject(collision_object10);

  

      // Shutdown ROS 
    rclcpp::spin(node);
    rclcpp::shutdown(); 

    return 0;
}