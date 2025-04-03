# Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # AudioPlayBackNode
    playback_node = ComposableNode(
        package='qrb_ros_audio_common',
        plugin='qrb_ros::audio_common::AudioCommonNode',
        name='audio_playback_node',
        parameters=[{
                'Stream_type': 'playback',
                'action_name': 'ros_audio_playback',
                'topic_name': 'qrb_audiodata',
            }]
    )

    # AudioCaptureNode
    capture_node = ComposableNode(
        package='qrb_ros_audio_common',
        plugin='qrb_ros::audio_common::AudioCommonNode',
        name='audio_capture_node',
        parameters=[{
                'Stream_type': 'capture',
                'action_name': 'ros_audio_capture',
                'topic_name': 'qrb_audiodata',
            }]
    )

    container = ComposableNodeContainer(
        name='audio_node_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            playback_node,
            capture_node
        ],
        output='screen'
    )

    return LaunchDescription([
        container
    ])
