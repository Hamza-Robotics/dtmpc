# ~/your_ros2_ws/src/your_package/launch/mpc_and_vicon_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mpc',  
            executable='mpc_controller_robot1.py',
            name='mpc_controller_robot1',
            output='screen',
            remappings=[
                ('/cmd_vel', '/robot1/cmd_vel')
            ]
        ),
        Node(
            package='vicon_modules',  
            executable='publish_state.py',
            name='publish_state',
            output='screen'
        )
    ])
