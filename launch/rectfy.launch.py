import os
import launch
import launch_ros
import launch.actions
from launch.conditions import LaunchConfigurationNotEquals, LaunchConfigurationEquals
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_share = get_package_share_directory('mynteye_get_data')

    # stereo_image_proc_node = launch_ros.actions.Node(
    #     package='stereo_image_proc',
    #     executable='stereo_image_proc',
    #     name='stereo_image_proc',
    #     output='screen',
    #     parameters=[os.path.join(
    #         pkg_share,
    #         'config',
    #         'stereo_image_proc.yaml'
    #     )],
    #     remappings=[('left/image_raw',    '/mynteye/left/image_raw'),
    #                 ('left/camera_info',  '/mynteye/left/camera_info'),
    #                 ('right/image_raw',   '/mynteye/right/image_raw'),
    #                 ('right/camera_info', '/mynteye/right/camera_info')],
    # )

    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_mono_node_left',
            # Remap subscribers and publishers
            remappings=[
                ('/image', '/mynteye/left/image_raw'),
                ('/camera_info', '/mynteye/left/camera_info'),
                ('/image_rect', '/left/image_rect')
            ],
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_mono_node_right',
            # Remap subscribers and publishers
            remappings=[
                ('/image', '/mynteye/right/image_raw'),
                ('/camera_info', '/mynteye/right/camera_info'),
                ('/image_rect', '/right/image_rect')
            ],
        ),
    ]

    image_processing_container = ComposableNodeContainer(
        name='image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen'
    )

    aruco_left = launch_ros.actions.Node(
        name='aruco_left_node',
        package = 'ros2_aruco',
        executable = 'aruco_node',
        parameters = [os.path.join(
            pkg_share,
            'config',
            'aruco.yaml')],
        remappings=[('/aruco_markers', '/left/aruco_markers'),
                    ('/aruco_poses', '/left/aruco_poses')]
    )

    aruco_right = launch_ros.actions.Node(
        name='aruco_right_node',
        package = 'ros2_aruco',
        executable = 'aruco_node',
        parameters = [os.path.join(
            pkg_share,
            'config',
            'aruco.yaml')],
        remappings=[('/aruco_markers', '/right/aruco_markers'),
                    ('/aruco_poses', '/right/aruco_poses')]
    )

    return launch.LaunchDescription([
        image_processing_container,
        aruco_left,
        aruco_right
        ])
