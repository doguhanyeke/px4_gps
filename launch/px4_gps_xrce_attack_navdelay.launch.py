#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Kartik Anand Pant"
__contact__ = "kpant14@gmail.com"

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    px4_ns = LaunchConfiguration('px4_ns')
    gz_world_name = LaunchConfiguration('gz_world_name')
    gz_model_name = LaunchConfiguration('gz_model_name')
    gz_spoofer_model_name = LaunchConfiguration('gz_spoofer_model_name')
    gps_delay = LaunchConfiguration('gps_delay')

    px4_ns_arg = DeclareLaunchArgument(
        'px4_ns',
        default_value='px4_1'
    )
    gz_world_name_arg = DeclareLaunchArgument(
        'gz_world_name',
        default_value='AbuDhabi'
    )
    gz_model_name_arg = DeclareLaunchArgument(
        'gz_model_name',
        default_value='x500_1'
    )
    gz_spoofer_model_name_arg = DeclareLaunchArgument(
        'gz_spoofer_model_name',
        default_value='spoofer'
    )
    gps_delay_arg = DeclareLaunchArgument(
        'gps_delay',
        default_value='10'
    )

    falsi_config = os.path.join(
        get_package_share_directory('px4_gps'),
        'config',
        'falsi_navdelay.yaml'
    )
    mavros_gps_node = Node(
           package='px4_gps',
           executable='px4_gps_pub_xrce',
           name='px4_gps_pub_xrce',
           parameters=[
                {'px4_ns': px4_ns},
                {'gz_world_name': gz_world_name},
                {'gz_model_name': gz_model_name},
                {'gz_spoofer_model_name': gz_spoofer_model_name},
                {'gps_delay': gps_delay},
                falsi_config
            ]
    )

    return LaunchDescription([
        px4_ns_arg,
        gz_world_name_arg,
        gz_model_name_arg,
        gz_spoofer_model_name_arg,
        gps_delay_arg,
        mavros_gps_node,
    ])
