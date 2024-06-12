import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='simulation_numerical' #<--- CHANGE ME
    pkg_path = os.path.join(get_package_share_directory(package_name))


    robot1 = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('vicon_modules'),'launch','sim_robot1.launch.py'
                    )])
        )
    robot2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('vicon_modules'),'launch','sim_robot2.launch.py'
                )])
    )

    robot3 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('vicon_modules'),'launch','sim_robot3.launch.py'
                )])
    )



    rvizfile=os.path.join(pkg_path,'config','rvizconfig.rviz')

    node_rviz2= Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rvizfile]
    )
  
    obstacle_node= Node(
    package='simulation_numerical',
    executable='obstacle.py'
) 
    
    formation_node= Node(
    package='simulation_numerical',
    executable='formation_plotter.py'
) 
    
    solution_node= Node(
    package='vicon_modules',
    executable='vizualize_solution.py'
) 

    trajectory_node= Node(
    package='vicon_modules',
    executable='vizualize_trajectory.py'
)
    
    show_state_node= Node(
    package='vicon_modules',
    executable='publish_state.py')

    return LaunchDescription([
        robot1,
        robot2,
        robot3,
        node_rviz2,
        obstacle_node,
        formation_node,
        solution_node,
        trajectory_node,
        show_state_node

    ])