from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')
    px4_gps_node = Node(
           package='px4_gps',
           executable='px4_gps_sim_pub',
           name='px4_gps_sim_pub'
        )
    gz_true_pos_node = Node(
            package='offboard_detector',
            executable='gz_true_pos_pub',
            output ='screen'
        )
    offboard_detector_node = Node(
            package='offboard_detector',
            executable='observer_detector.py',
        )

    return LaunchDescription([
        px4_gps_node,
        gz_true_pos_node,
        offboard_detector_node, 
    ])
