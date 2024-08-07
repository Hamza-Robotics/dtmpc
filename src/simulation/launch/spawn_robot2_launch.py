# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License foros2r the specific language governing permissions and
# limitations under the License.

import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

robot_name="robot2"
robot_init=[1,1]
def generate_launch_description():
    # Get the urdf file
    urdf_file_name = 'autogen/'+robot_name+'.urdf'
    #urdf_file_name = 'model.urdf'
    sdf_path = os.path.join(
        get_package_share_directory('simulation'),
        'models',
        robot_name+'.sdf'
    )

    urdf = os.path.join(
    get_package_share_directory('simulation'),
    'models',
    urdf_file_name)
    
    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')
    
    namespace = [ '/' + robot_name]
    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-file', sdf_path,
            '-x', str(robot_init[0]),
            '-y', str(robot_init[1]),
            '-z', '0.01',
            '-robot_namespace', namespace
        ],
        output='screen',
    )

 

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[rsp_params, {'use_sim_time': True}],
        
        remappings = [('/joint_states', '/'+robot_name+'/joint_states'),('/robot_description', '/'+robot_name+'/robot_description')]

    )


    ld = LaunchDescription()



    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(node_robot_state_publisher)

    return ld