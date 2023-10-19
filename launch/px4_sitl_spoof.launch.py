from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')
    px4_gps_node = Node(
           package='px4_gps',
           executable='px4_gps_sim_pub',
           name='px4_gps_real_pub'
        )
    entity_service = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/AbuDhabi/set_pose@ros_gz_interfaces/srv/SetEntityPose']
    )

    spoofer_node = Node(
           package='px4_offboard',
           executable='spoofer_gz_stream',
           name='spoofer_gz_stream'
    ) 
    return LaunchDescription([
        px4_gps_node,
        entity_service,
        spoofer_node,
    ])
