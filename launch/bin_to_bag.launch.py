# MIT License
# Copyright (c) 2026 fengruhua
# See LICENSE file for details.

# This launch file converts the raw binary point cloud data into ROS2 bag files for easier playback and analysis. 
# It assumes that the `bin_to_bag_node` executable takes two arguments: 
# 1, The input directory containing the binary files.
# 2, The output directory for the generated bag files.

# You can run this launch file using:
#     ros2 launch doppler_icp bin_to_bag.launch.py

# The output bag structure will be as follows:
#     REPOSITORY_ROOT/dataset/dataset-rosbag/
#     ├── sequence_01/
#     ├── sequence_01/
#     └── ...

# The PointCloud2 messages in the bag files will have the following fields:
#     - x, y, z: float32 (point coordinates)
#     - intensity: float32 (Doppler velocity)

# The dataset structure is as follows:

#     REPOSITORY_ROOT/dataset/
#     ├── sequence_01/
#     │   ├── point_clouds/
#     │   │   ├── 00001.bin  # N * (3 + 1) float32 bytes containing XYZ points
#     │   │   ├── 00002.bin  # and Doppler velocities.
#     │   │   └── ...
#     │   ├── calibration.json
#     │   └── ref_poses.txt  # N reference poses with timestamps in TUM format.
#     ├── sequence_02/
#     │   └── ...
#     └── ...

from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'run',
                'doppler_icp',
                'bin_to_bag_node',
                # the input directory containing the binary files
                './Doppler-ICP-Cpp/dataset/carla-town05-curved-walls/point_clouds',
                # the output directory for the generated bag files
                './Doppler-ICP-Cpp/dataset/dataset-rosbag/carla-town05-curved-walls'
            ],
            output='screen'
        )
    ])