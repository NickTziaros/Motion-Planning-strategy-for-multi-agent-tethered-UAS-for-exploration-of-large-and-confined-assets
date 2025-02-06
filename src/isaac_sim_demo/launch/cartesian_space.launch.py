from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("isaac_twin", package_name="prismatic_moveit_config")
        .robot_description(file_path="config/isaac_twin.urdf.xacro")
        .robot_description_semantic(file_path="config/isaac_twin.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl","stomp"])
        .to_moveit_configs()
        )
    ompl_planning_pipeline_config = { 
    'planning_plugin' : ['ompl_interface/OMPLPlanner'],
    'default_planning_pipeline': 'ompl',
    'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
    'start_state_max_bounds_error' : 0.1 ,   
     }
    cpp_demo =  Node(
                package="isaac_sim_demo",
                executable="cartesian",
                name="cartesian",
                output="screen",
                parameters=[moveit_config.to_dict(),
                ompl_planning_pipeline_config],
            )


    return LaunchDescription([

        cpp_demo
        
    ])