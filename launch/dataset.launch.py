# MIT License
# Copyright (c) 2026 fengruhua
# See LICENSE file for details.

# This launch file plays back the dataset of Doppler point clouds for visualization and analysis.
# It assumes that the `dataset_node` executable takes parameters for the data path, frame rate, point topic, frame ID, and whether to loop the playback.

# You can run this launch file using:
#     ros2 launch doppler_icp dataset.launch.py

# The PointCloud2 messages will have the following fields:
#     - x, y, z: float32 (point coordinates)
#     - doppler: float32 (Doppler velocity)

# "data_path" : The directory containing the binary point cloud files. 

# "frame_rate" : The rate at which to publish the point cloud messages 
# (e.g., 10.0 for 10 Hz).

# "point_topic" : The ROS topic name to publish the PointCloud2 messages 
# (e.g., "/points").

# "frame_id" : The frame ID to use in the header of the PointCloud2 messages 
# (e.g., "/map").

# "loop" : Whether to loop the playback of the dataset 
# (true or false).

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.actions import Shutdown
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    rviz_config = os.path.join(
        get_package_share_directory('doppler_icp'),
        'rviz',
        'dataset.rviz'
    )

    dataset_node = Node(
        package='doppler_icp',
        executable='dataset_node',
        name='doppler_dataset',
        parameters=[
            {'data_path': './Doppler-ICP-Cpp/dataset/carla-town04-straight-walls/point_clouds'},
            {'frame_rate': 0.1},
            {'point_topic': '/points'},
            {'frame_id': '/map'},
            {'loop': False}
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],   # 指定rviz配置
        output='screen'
    )

    delayed_neu_node = TimerAction(period=1.0, actions=[dataset_node])

    shutdown_event = OnProcessExit(
        target_action=dataset_node,
        on_exit=[Shutdown()]
    )

    return LaunchDescription([
        delayed_neu_node,
        # rviz_node,
        RegisterEventHandler(shutdown_event)
    ])