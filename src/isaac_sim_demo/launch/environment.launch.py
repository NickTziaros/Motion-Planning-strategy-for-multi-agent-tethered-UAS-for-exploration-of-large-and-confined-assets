from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder

import os


    
def generate_launch_description():
    
    # moveit_config = MoveItConfigsBuilder("isaac_twin", package_name="test_moveit_config").to_moveit_configs()
    
    my_package_dir = get_package_share_directory('isaac_sim_demo')
    moveit_dir= get_package_share_directory('prismatic_moveit_config')
    tf_to_pos = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         my_package_dir, 'launch'),
         '/tf_to_pos_launch.py'])
      )
    
    # moveit_demo = IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource([os.path.join(
    #      moveit_dir, 'launch'),
    #      '/demo.launch.py'])
    #   )

    moveit_demo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         my_package_dir, 'launch'),
         '/test_launch.py'])
      )



    return LaunchDescription([
        moveit_demo,
        tf_to_pos,

        Node(
                    package="isaac_sim_demo",
                    executable="ompl_lifecycle",
                    name="ompl_controller_node",
                    namespace="Quadrotor_1",
                    # output="screen",
                    # emulate_tty=True,
                    parameters=[{"Quadrotor": "Quadrotor_1"}]
                ),
        Node(
                    package="isaac_sim_demo",
                    executable="ompl_lifecycle",
                    name="ompl_controller_node",
                    namespace="Quadrotor_2",
                    # output="screen",
                    # emulate_tty=True,
                    parameters=[
                        {"Quadrotor": "Quadrotor_2"}]
                ),
        Node(
                    package="isaac_sim_demo",
                    executable="ompl_lifecycle",
                    name="ompl_controller_node",
                    namespace="Quadrotor_3",
                    # output="screen",
                    # emulate_tty=True,
                    parameters=[
                        {"Quadrotor": "Quadrotor_3"}]
                ),
        Node(
                    package="isaac_sim_demo",
                    executable="ompl_lifecycle",
                    name="ompl_controller_node",
                    namespace="Quadrotor_4",
                    # output="screen",
                    # emulate_tty=True,
                    parameters=[
                        {"Quadrotor": "Quadrotor_4"}]
                )

    ])
