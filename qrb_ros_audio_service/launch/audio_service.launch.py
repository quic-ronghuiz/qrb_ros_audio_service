# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([ComposableNodeContainer(
        output='screen',
        name='component_audio_service_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='qrb_ros_audio_service',
                plugin='qrb_ros::audio_service::AudioServer',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ]
    )])