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
def generate_launch_description():
    urdf_file_name = 'autogen/'+robot_name+'.urdf'
    urdf = os.path.join(
    get_package_share_directory('simulation_numerical'),'models',urdf_file_name)
    # Declare the launch arguments

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
    simulation_num=Node(
        package='simulation_numerical',
        executable='num_sim_'+robot_name+'.py',
        name='numsim'+robot_name,
        output='screen')



    ld = LaunchDescription()
    ld.add_action(node_robot_state_publisher)
    ld.add_action(simulation_num)

    return ld