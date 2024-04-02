import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description(): 
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r /home/kap/ros2_ws_gz/src/world_sdf/AbuDhabi_MR.sdf'
        }.items(),
    )
    
    mocap_node = Node(
            package='qualisys_mocap',
            executable='qualisys_node', 		
    )
    
    px4_gps_node = Node(
           package='px4_gps',
           executable='px4_gps_real_pub',
           name='px4_gps_real_pub',
    )
    
    entity_service = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/AbuDhabi/set_pose@ros_gz_interfaces/srv/SetEntityPose']
    )

    model_pose = Node(
           package='px4_offboard',
           executable='mocap_gz_stream',
           name='mocap_gz_stream'
    ) 
    
    

    onboard_camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera'],
        output='screen'
    )


    return LaunchDescription([
        gz_sim,
        px4_gps_node,
        entity_service,
        model_pose,
        mocap_node,
        onboard_camera_bridge,
        #rviz_node
    ])
