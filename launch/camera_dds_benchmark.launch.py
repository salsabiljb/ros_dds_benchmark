from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbag2_transport',
            executable='play',
            name='rosbag_play',
            arguments=['bags/4k_camera_60fps.bag'],
            output='screen'
        ),
        Node(
            package='ros2_dds_camera_benchmark',
            executable='image_subscriber_node',
            name='image_subscriber',
            output='screen'
        )
    ])
