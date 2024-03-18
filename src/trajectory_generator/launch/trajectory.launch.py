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



    rvizfile=os.path.join(pkg_path,'config','rvizconfig.rviz')


    waypointsender= Node(
        package='trajectory_generator',
        executable='waypointsender.py',
        name='waypoint_onde')

    trajectory= Node(
        package='trajectory_generator',
        executable='trajectory.py',
        name='trajectory_node')


  
    
    
    return LaunchDescription([
        trajectory,
        waypointsender
  
    ])