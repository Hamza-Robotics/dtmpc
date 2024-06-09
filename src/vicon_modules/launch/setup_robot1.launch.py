# ~/your_ros2_ws/src/your_package/launch/mpc_and_vicon_launch.py


import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='simulation_numerical' #<--- CHANGE ME
    pkg_path = os.path.join(get_package_share_directory(package_name))


    robot_bring_up = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('turtlebot3_bringup'),'launch','robot.launch.py'
                    )])
        )



   
    ovicon_state= Node(
    package='vicon_modules',
    executable='publish_state.py',
    output='screen',
) 
    
    obstacle_node= Node(
    package='simulation_numerical',
    executable='obstacle.py'
) 
    

    mpc1=Node(
    package='mpc',
    executable='mpc_controller_robot1.py',
               remappings=[
                ('/robot1/cmd_vel', '/cmd_vel')],
                 output='screen'

) 
    
    
    return LaunchDescription([
        robot_bring_up,
        ovicon_state,
        mpc1,
        obstacle_node
    ])


