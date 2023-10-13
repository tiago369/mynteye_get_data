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

    left_cam_info_node = launch_ros.actions.Node(
        name='cam_info_pub_node_left',
        package = 'mynteye_get_data',
        executable = 'cam_info_pub',
        parameters = [os.path.join(
            pkg_share,
            'config',
            'camera_info.yaml')],
        remappings=[('camera_info_sub', '/mynteye/left/camera_info'),
                    ('camera_info_pub', '/left/camera_info')])
    
    right_cam_info_node = launch_ros.actions.Node(
        name='cam_info_pub_node_right',
        package = 'mynteye_get_data',
        executable = 'cam_info_pub',
        parameters = [os.path.join(
            pkg_share,
            'config',
            'camera_info.yaml')],
        remappings=[('camera_info_sub', '/mynteye/right/camera_info'),
                    ('camera_info_pub', '/right/camera_info')])

    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_mono_node_left',
            # Remap subscribers and publishers
            remappings=[
                ('/image', '/mynteye/left/image_raw'),
                ('/camera_info', '/left/camera_info'),
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
                ('/camera_info', '/right/camera_info'),
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


    pose_estimator_node = launch_ros.actions.Node(
        name='pose_estimator_node',
        package='mynteye_get_data',
        executable='aruco_mean',
        parameters=[os.path.join(
            pkg_share,
            'config',
            'pose_estimator.yaml')],
        remappings=[('/left_topic', '/left/aruco_markers'),
                    ('/right_topic', '/right/aruco_markers')]
    )

    return launch.LaunchDescription([
        left_cam_info_node,
        right_cam_info_node,
        image_processing_container,
        aruco_left,
        aruco_right,
        pose_estimator_node
        ])
