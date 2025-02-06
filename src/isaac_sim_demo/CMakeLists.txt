cmake_minimum_required(VERSION 3.8)
project(isaac_sim_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ompl REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(four_drone_demo REQUIRED)

# find_package(joint_state_publisher REQUIRED)
# find_package(joint_state_publisher_gui REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
  /home/ubuntu/ros2_ws/build
  ros2_ws/install/custom_interfaces/include/custom_interfaces/custom_interfaces
  /home/ubuntu/ros2_ws/src/isaac_sim_demo/include/isaac_sim_demo
  /opt/ros/humble/tf2_geometry_msgs
  /opt/ros/humble/include/ompl-1.6
  ${EIGEN3_INCLUDE_DIRS}
  ${colcon_INCLUDE_DIRS}
)

file(GLOB RRT_controller_sources
    src/RRT_controller.cpp
)

file(GLOB RRT_controller_headers
    include/RRT_controller/*.hpp
)


add_library(${PROJECT_NAME}_library STATIC ${RRT_controller_sources} ${RRT_controller_headers} 
)

ament_target_dependencies(
  ${PROJECT_NAME}_library
  rclcpp
  ompl
  four_drone_demo
  nav_msgs
  lifecycle_msgs
  geometry_msgs
  visualization_msgs
  custom_interfaces
  std_srvs
  tf2_geometry_msgs
  moveit_ros_planning_interface 
  moveit_visual_tools
  rclcpp_lifecycle 
)


# add_library(RRT_controller
#   src/RRT_controller.cpp
# )




add_executable(StateSubscriber_node src/StateSubscriber_node.cpp)
ament_target_dependencies(StateSubscriber_node rclcpp geometry_msgs)


add_executable(markers_node src/markers.cpp)
ament_target_dependencies(markers_node rclcpp geometry_msgs visualization_msgs)

add_executable(ompl_controller_node src/ompl_controller_node.cpp)
ament_target_dependencies(ompl_controller_node rclcpp geometry_msgs visualization_msgs nav_msgs ompl custom_interfaces)  
target_link_libraries(ompl_controller_node ${PROJECT_NAME}_library ${OMPL_LIBRARIES})


add_executable(visualization_node src/visualization.cpp)
ament_target_dependencies(visualization_node  rclcpp geometry_msgs visualization_msgs nav_msgs std_srvs custom_interfaces) 


add_executable(tf_to_pos_node src/tf_to_pos.cpp)
ament_target_dependencies(tf_to_pos_node  rclcpp geometry_msgs tf2_ros nav_msgs tf2_geometry_msgs custom_interfaces) 

add_executable(moveit_test_node src/moveit_demo.cpp)
ament_target_dependencies(moveit_test_node  moveit_ros_planning_interface  moveit_visual_tools four_drone_demo)

add_executable(joint_space src/joint_space.cpp)
ament_target_dependencies(joint_space  moveit_ros_planning_interface  moveit_visual_tools rclcpp tf2_geometry_msgs nav_msgs)

add_executable(cartesian src/cartesian.cpp)
ament_target_dependencies(cartesian  moveit_ros_planning_interface  moveit_visual_tools rclcpp tf2_geometry_msgs nav_msgs)


add_executable(spawn_scene src/spawn_scene.cpp)
ament_target_dependencies(spawn_scene  moveit_ros_planning_interface  moveit_visual_tools rclcpp tf2_geometry_msgs)

add_executable(ompl_lifecycle src/ompl_lifecycle.cpp)
ament_target_dependencies(ompl_lifecycle  rclcpp std_msgs std_srvs lifecycle_msgs nav_msgs visualization_msgs rclcpp_lifecycle  ompl custom_interfaces ) 
target_link_libraries(ompl_lifecycle ${PROJECT_NAME}_library ${OMPL_LIBRARIES})


add_executable(drone_marker_publisher src/drone_marker_publisher.cpp)
ament_target_dependencies(drone_marker_publisher visualization_msgs rclcpp tf2_geometry_msgs)


# when the ompl package is isntalled it provides the variable ${OMPL_LIBRARIES}
# Install launch files.
install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}/
)
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(TARGETS
  tf_to_pos_node
  StateSubscriber_node
  markers_node
  ompl_controller_node
  visualization_node
  moveit_test_node
  ompl_lifecycle
  joint_space
  spawn_scene
  cartesian
  drone_marker_publisher
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
