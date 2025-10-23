from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import tempfile


def generate_launch_description():

    # Declare the namespace argument (it can be provided when launching)
    namespace=LaunchConfiguration('namespace')
    server=LaunchConfiguration('server')
    port=LaunchConfiguration('port')
    frame_id=LaunchConfiguration('frame_id')
    update_freq=LaunchConfiguration('update_freq')
    refresh_freq=LaunchConfiguration('refresh_freq')
    multi_sensor=LaunchConfiguration('multi_sensor')
    use_vrpn_timestamps=LaunchConfiguration('use_vrpn_timestamps')
    mocap_use=LaunchConfiguration('mocap_use')
    vicon_object_name=LaunchConfiguration('vicon_object_name')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('server',default_value='localhost'),
        DeclareLaunchArgument('port',default_value='3883'),
        DeclareLaunchArgument('frame_id',default_value='world'),
        DeclareLaunchArgument('update_freq',default_value='100.0'),
        DeclareLaunchArgument('refresh_freq',default_value='1.0'),
        DeclareLaunchArgument('multi_sensor',default_value='false'),
        DeclareLaunchArgument('use_vrpn_timestamps',default_value='false'),
        DeclareLaunchArgument('vicon_object_name', default_value='Aira'),
        DeclareLaunchArgument('mocap_use',default_value='2',description='Intended use of the mocap data: 1. EKF sensor fusion 2. Ground truth reference'),

        Node(
            package='vrpn_mocap',
            executable='client_node',
            name='vrpn_mocap_client_node',
            namespace='vrpn_mocap',
            parameters=[
                {'server': server},
                {'port': port},
                {'frame_id': frame_id},
                {'update_freq': update_freq},
                {'refresh_freq': refresh_freq},
                {'multi_sensor': multi_sensor},
                {'use_vrpn_timestamps': use_vrpn_timestamps}
            ]
        ),
        Node(
            package='vision',
            executable='vicon_bridge',
            name='vicon_bridge',
            namespace=namespace,
            parameters=[
                {'mocap_use': mocap_use},
                {'vicon_object_name': vicon_object_name}
            ],
            output='screen'
        ),
    ])

