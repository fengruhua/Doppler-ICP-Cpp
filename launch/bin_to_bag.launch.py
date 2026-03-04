from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'run',
                'doppler_icp',
                'bin_to_bag_node',
                '/home/fengruhua/Doppler/Doppler-ICP-Cpp/dataset/carla-town05-curved-walls/point_clouds',
                '/home/fengruhua/Doppler/Doppler-ICP-Cpp/dataset-rosbag/carla-town05-curved-walls'
            ],
            output='screen'
        )
    ])