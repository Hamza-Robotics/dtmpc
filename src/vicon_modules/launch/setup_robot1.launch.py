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




    robot_bring_up = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('turtlebot3_bringup'),'launch','robot.launch.py'
                    )])
        )


    

    mpc1=Node(
    package='mpc',
    executable='mpc_real_robot1.py',
    output='screen',

) 
    
    
    return LaunchDescription([
    
        robot_bring_up,

        mpc1
    ])


