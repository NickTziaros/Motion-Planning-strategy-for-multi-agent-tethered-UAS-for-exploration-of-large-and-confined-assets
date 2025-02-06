import os
import yaml
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
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





    moveit_config = (
        MoveItConfigsBuilder("isaac_twin", package_name="prismatic_moveit_config")
        .robot_description(file_path="config/isaac_twin.urdf.xacro")
        .robot_description_semantic(file_path="config/isaac_twin.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl", "stomp"])
        .to_moveit_configs()
    )   
    package_path = get_package_share_directory('prismatic_moveit_config')


    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
    ]

    rviz= Node(
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', os.path.join(package_path, 'config', 'mark_state.rviz')],
                    parameters=rviz_parameters,
                )    



    move_group_configuration = {
            "publish_robot_description_semantic": True,
            "allow_trajectory_execution": True,

            "publish_monitored_planning_scene": True,
            "providePlanningSceneService": True,

            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
            "monitor_dynamics": False,
            
        }



    trajectory_execution = {'moveit_manage_controllers': True,
                            'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                            'trajectory_execution.allowed_goal_duration_margin': 0.5,
                            'trajectory_execution.allowed_start_tolerance': 0.01}



    ompl_planning_pipeline_config = { 
        # 'planning_plugin' : 'chomp_interface/CHOMPPlanner',
        
        # 'planning_plugin' : 'ompl_interface/OMPLPlanner',

        # change this to select planner
        'default_planning_pipeline': 'stomp',
        'enforce_joint_model_state_space': 'False',
        'request_adapters' : "default_planner_request_adapters/  default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
        'start_state_max_bounds_error' : 0.1 }

            
    octomap_config = {'octomap_frame': 'World', 
                      'octomap_resolution': 0.1,
                      'max_range': 10.0}

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
        move_group_configuration,
        octomap_config,
        ompl_planning_pipeline_config,
        trajectory_execution
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )


    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        respawn=True,
        parameters=[            moveit_config.robot_description,
            {
                "publish_frequency": 15.0,
            }],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("prismatic_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["Group1_controller", "-c", "/controller_manager"],
    )



    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )



    # octomap_updater_config = load_yaml('isaac_sim_demo', 'config/sensors_3d.yaml')



    joint_state_publisher=Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[moveit_config.robot_description]
        )

    world_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'World', 'base_link'])
    spawn_scene = Node(
            package='isaac_sim_demo',
            executable='spawn_scene',
            name='spawn_scene',
            )












    return LaunchDescription(
        [   world_tf,
            robot_state_publisher,
            rviz,
            run_move_group_node,
            ros2_control_node,
            panda_arm_controller_spawner,
            joint_state_broadcaster_spawner,
            spawn_scene,


            
 
        ]
    )
    # return LaunchDescription([ run_move_group_node, robot_state_publisher,joint_state_publisher,fake ])