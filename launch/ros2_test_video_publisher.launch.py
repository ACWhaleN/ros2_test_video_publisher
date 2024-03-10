# launch/ros2_test_video_publisher.launch.py
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate the launch description for the image_pub node.

    Returns:
        LaunchDescription: The launch description object.
    """
    params_file = os.path.join(get_package_share_directory(
        'ros2_test_video_publisher'), 'config', 'default.yaml')

    ros2_test_video_publisher_node = Node(
        package='ros2_test_video_publisher',
        executable='ros2_test_video_publisher_node',
        namespace='test',
        name='image_publisher',
        output="screen",
        emulate_tty=True,
        parameters=[params_file]
    )

    return LaunchDescription([
        ros2_test_video_publisher_node 
    ])
