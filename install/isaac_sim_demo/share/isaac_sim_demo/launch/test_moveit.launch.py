from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
import yaml
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import os
from moveit_configs_utils import MoveItConfigsBuilder

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None
    
def generate_launch_description():
    
    ompl_planning_pipeline_config = { 'moveit_test_node' : 
    {
    'planning_plugin' : 'ompl_interface/OMPLPlanner',
    'default_planning_pipeline': 'chomp',

    'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
    'start_state_max_bounds_error' : 0.1    
     },
    'Group1': {'default_planner_config' : 'AnytimePathShorteningConfigDefault'}
    }
    moveit_config = (
        MoveItConfigsBuilder("isaac_twin",package_name="four_drone_demo")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp"])
        .robot_description_kinematics(file_path="config/kinematics.yaml")

        .robot_description(file_path="config/isaac_twin.urdf.xacro")
        .robot_description_semantic(file_path="config/isaac_twin.srdf")
        .to_moveit_configs()
    )

    # kinematics_yaml = load_yaml('four_drone_demo', 'config/kinematics.yaml')
    # robot_description_kinematics = { 'robot_description_kinematics' : kinematics_yaml }
    moveit_test   =   Node(
                    package="isaac_sim_demo",
                    executable="moveit_test_node",
                    name="moveit_test_node",
                    parameters=[moveit_config.to_dict(),
                    ompl_planning_pipeline_config
                        ]
                )






    return LaunchDescription([

        moveit_test

    ])
