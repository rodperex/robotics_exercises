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

    yolo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('yolo_bringup'),
            'launch',
            'yolo.launch.py')),
        launch_arguments={
            'input_image_topic': '/rgbd_camera/image',
            'input_depth_topic': '/rgbd_camera/depth_image',
        }.items()
    )

    detection_2d_cmd = Node(
        package='camera',
        executable='yolo_detection',
        name='yolo_detection',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('input_detection', '/yolo/detections'),
            ('output_detection_2d', '/detection_2d')]
    )

    convert_2d_3d = Node(
        package='camera',
        executable='detection_2d_to_3d_depth',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('input_depth', '/rgbd_camera/depth_image'),
            ('input_detection_2d', 'detection_2d'),
            ('camera_info', '/rgbd_camera/camera_info'),
            ('output_detection_3d', 'detection_3d')]
    )

    detectionpub_cmd = Node(
        package='follow_person',
        executable='detection_tf_publisher',
        name='detection_tf_publisher',
        output='screen',
        parameters=[param_file, {'use_sim_time': True}],
        remappings=[('output_detection_3d', '/detection_3d')]
    )

    rviz2_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    ld = LaunchDescription()

    ld.add_action(yolo_cmd)
    ld.add_action(detection_2d_cmd)
    ld.add_action(convert_2d_3d)
    ld.add_action(detectionpub_cmd)
    ld.add_action(rviz2_cmd)

    return ld
