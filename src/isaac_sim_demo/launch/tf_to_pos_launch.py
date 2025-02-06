from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os


    
def generate_launch_description():
    
    my_package_dir = get_package_share_directory('isaac_sim_demo')
    
    return LaunchDescription([

 
        Node(
                    package='isaac_sim_demo',
                    executable='tf_to_pos_node',
                    namespace='Quadrotor_1',
                    parameters=[{"Quad_Number": "1"}]
                ),
        Node(
                    package='isaac_sim_demo',
                    executable='tf_to_pos_node',
                    namespace='Quadrotor_2',
                    parameters=[{"Quad_Number": "2"}]
                ) ,
        Node(
                    package='isaac_sim_demo',
                    executable='tf_to_pos_node',
                    namespace='Quadrotor_3',
                    parameters=[{"Quad_Number": "3"}]
                ) ,
        Node(
                    package='isaac_sim_demo',
                    executable='tf_to_pos_node',
                    namespace='Quadrotor_4',
                    parameters=[{"Quad_Number": "4"}]
                )                 

    ])
