from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import tempfile


def generate_launch_description():

    # Declare the namespace argument (it can be provided when launching)
    namespace = LaunchConfiguration('namespace', default='px4_offboard')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='aira1',
            description='Namespace of the nodes'
        ),
        Node(
            package='vision',
            namespace=namespace,
            executable='vio_bridge_px4',
            name='vio_bridge',
            parameters=[
                {'namespace': namespace}
            ]
        ),
        Node(
            package='visualizer',
            namespace=namespace,
            executable='visualizer',
            name='visualizer',
            parameters=[
                {'namespace': namespace}
            ]
        ),
        Node(
            package='offboard_controller',
            namespace=namespace,
            executable='oc_posctl',
            name='control',
            parameters= [
                {'radius': 10.0},
                {'altitude': 5.0},
                {'omega': 0.5},
                {'namespace': namespace}
            ]
        ),
        # OpaqueFunction(function=launch_setup),
    ])

# def patch_rviz_config(original_config_path, namespace):
#     """
#     Patch the RViz configuration file to replace the namespace placeholder with the actual namespace.
#     """
#     with open(original_config_path, 'r') as f:
#         content = f.read()

#     # Replace placeholder with actual namespace
#     content = content.replace('__NS__', f'/{namespace}' if namespace else '')
    
#     # Write to temporary file
#     tmp_rviz_config = tempfile.NamedTemporaryFile(delete=False, suffix='.rviz')
#     tmp_rviz_config.write(content.encode('utf-8'))
#     tmp_rviz_config.close()

#     return tmp_rviz_config.name


# def launch_setup(context, *args, **kwargs):
#     """
#     Function to set up the launch context and patch the RViz configuration.
#     """
#     namespace = LaunchConfiguration('namespace').perform(context)
#     rviz_config_path = os.path.join(get_package_share_directory('px4_offboard'), 'visualize.rviz')
#     patched_config = patch_rviz_config(rviz_config_path, namespace)

#     return [
#         Node(
#             package='rviz2',
#             namespace='',
#             executable='rviz2',
#             name='rviz2',
#             arguments=['-d', patched_config]
#         )
#     ]