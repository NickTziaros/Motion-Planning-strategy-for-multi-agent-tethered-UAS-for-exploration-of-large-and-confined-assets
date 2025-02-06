from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


    
def generate_launch_description():
    
    my_package_dir = get_package_share_directory('isaac_sim_demo')
    
    return LaunchDescription([

        Node(
                    package="isaac_sim_demo",
                    executable="visualization_node",
                    name="visualization_node",
                    output="screen",
                    emulate_tty=True,
                ),  

                

        Node(
                    package="isaac_sim_demo",
                    executable="ompl_controller_node",
                    name="ompl_controller_node_1",
                    output="screen",
                    emulate_tty=True,
                    parameters=[{"Quadrotor": "Quadrotor_1"}]
                ),
        Node(
                    package="isaac_sim_demo",
                    executable="ompl_controller_node",
                    name="ompl_controller_node_2",
                    output="screen",
                    emulate_tty=True,
                    parameters=[
                        {"Quadrotor": "Quadrotor_2"}]
                ),
        Node(
                    package="isaac_sim_demo",
                    executable="ompl_controller_node",
                    name="ompl_controller_node_3",
                    output="screen",
                    emulate_tty=True,
                    parameters=[
                        {"Quadrotor": "Quadrotor_3"}]
                ),
        Node(
                    package="isaac_sim_demo",
                    executable="ompl_controller_node",
                    name="ompl_controller_node_4",
                    output="screen",
                    emulate_tty=True,
                    parameters=[
                        {"Quadrotor": "Quadrotor_4"}]
                ),
        Node(
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', os.path.join(my_package_dir, 'config', 'rviz_config.rviz')]
                ),
        Node(
                    package='isaac_sim_demo',
                    executable='tf_to_pos_node',
                    
                )                

                       





        # Node(package='isaac_sim_demo',
        #      executable='StateSubscriber_node',
        #      )
    ])
