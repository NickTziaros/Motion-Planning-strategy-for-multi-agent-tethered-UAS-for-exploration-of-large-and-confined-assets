from moveit_configs_utils import MoveItConfigsBuilder
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    print(absolute_file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("isaac_twin", package_name="four_drone_demo").to_moveit_configs()
    global_planner_param = load_yaml(
        "isaac_sim_demo", "config/global_planner.yaml"
    )
    local_planner_param = load_yaml(
        "isaac_sim_demo", "config/local_planner.yaml"
    )
    hybrid_planning_manager_param = load_yaml(
        "isaac_sim_demo", "config/hybrid_planning_manager.yaml"
    )

    # Generate launch description with multiple components
    container = ComposableNodeContainer(
        name="hybrid_planning_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::GlobalPlannerComponent",
                name="global_planner",
                parameters=[
                    global_planner_param,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                ],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::LocalPlannerComponent",
                name="local_planner",
                parameters=[
                    local_planner_param,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                ],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::HybridPlanningManager",
                name="hybrid_planning_manager",
                parameters=[hybrid_planning_manager_param],
            ),
        ],
        output="screen",
    )
    return LaunchDescription([container])