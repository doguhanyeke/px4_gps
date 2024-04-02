import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    px4_gps_publisher = Node(
           package='px4_gps',
           executable='px4_gps_sim_pub',
           name='px4_gps_sim_pub'
        )
    
    gz_true_pos_publisher = Node(
            package='offboard_detector',
            executable='gz_true_pos_pub',
            output ='screen'
        )
    
    offboard_detector = Node(
            package='offboard_detector',
            executable='observer_detector.py',
        )
    
    # plotjuggler = Node(
    #     package='plotjuggler',
    #     executable='plotjuggler',
    #     arguments=['--layout', [os.path.join(
    #             get_package_share_directory('px4_gps'),
    #             'config', 'plotjuggler_layout_onboard.xml')]]
    #     )
    
    foxglove_bridge = IncludeLaunchDescription(XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("px4_gps"),
                "launch/foxglove_bridge.launch",))
    )

    foxglove_studio = ExecuteProcess(cmd=["foxglove-studio"])

    cam0_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera_0/image_image'],
        output='screen'
    )    
    cam1_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera_1/image_image'],
        output='screen'
    )    
    cam2_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera_2/image_image'],
        output='screen'
    )    
    cam3_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera_3/image_image'],
        output='screen'
    )    
    cam4_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera_4/image_image'],
        output='screen'
    )    
    cam5_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera_5/image_image'],
        output='screen'
    )    
    cam6_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera_6/image_image'],
        output='screen'
    )    
    cam7_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera_7/image_image'],
        output='screen'
    )     




    return LaunchDescription([
        px4_gps_publisher,
        gz_true_pos_publisher,
        offboard_detector, 
        #plotjuggler,
        # cam0_bridge,
        # cam1_bridge,
        # cam2_bridge,
        # cam3_bridge,
        # cam4_bridge,
        # cam5_bridge,
        # cam6_bridge,
        # cam7_bridge,
        # foxglove_bridge,
        # foxglove_studio
    ])
