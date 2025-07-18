# Copyright 2024 Intelligent Robotics Lab
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
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('follow_person')
    param_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    
    followperson_cmd = Node(
        package='follow_person',
        executable='person_follower',
        name='person_follower',
        output='screen',
        parameters=[param_file, {'use_sim_time': True}],
        remappings=[('velocity', '/cmd_vel')]
    )


    ld = LaunchDescription()

    ld.add_action(followperson_cmd)

    return ld
