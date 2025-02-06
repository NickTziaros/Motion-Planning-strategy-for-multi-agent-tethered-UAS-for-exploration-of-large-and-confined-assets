from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

import os


    
def generate_launch_description():
    
    my_package_dir = get_package_share_directory('isaac_sim_demo')

    

    urdf_file_name = 'chain_twin.xacro'
    urdf = os.path.join(my_package_dir,
        'urdf',
        urdf_file_name)

    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
        'robot_description':Command(['xacro ', str(urdf)])
        
    }]

        ),        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', os.path.join(my_package_dir, 'config', 'robot_desc.rviz')]
                ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'World', 'base_link']
        )
    ])
                       





        # Node(package='isaac_sim_demo',
        #      executable='StateSubscriber_node',
        #      )

