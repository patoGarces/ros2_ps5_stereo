import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ros2_ps5_stereo.utilsClass import Resolutions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    laser_filter_yaml = os.path.join(
        get_package_share_directory('hoverrobot_navigation'),
        "config",
        'laser_filters.yaml'
    )

    # Nodo de cámaras, ros2_ps5_stereo y el ejecutable camera_node
    camera_node = Node(
        package='ros2_ps5_stereo',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'camera_resolution': Resolutions.RES_640x480_DOWNSAMPLED_8FPS.value,
            'roi_height': 50       #  crop de los frames de cameras,  No enviar si no se utiliza,    
        }]
    )

    # Nodo de rectificación para cámara izquierda
    rectify_left = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify_left',
        output='screen',
        remappings=[
            ('image', '/left/image_raw'),
            ('camera_info', '/left/camera_info'),
            ('image_rect', '/left/image_rect')  # topic rectificado que publica
        ],
        parameters=[{'approximate_sync': True}]
    )

    # Nodo de rectificación para cámara derecha
    rectify_right = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify_right',
        output='screen',
        remappings=[
            ('image', '/right/image_raw'),
            ('camera_info', '/right/camera_info'),
            ('image_rect', '/right/image_rect')
        ],
        parameters=[{'approximate_sync': True}]
    )

    disparity_node = Node(
        package='stereo_image_proc',
        executable='disparity_node',
        name='disparity_node',
        parameters=[{
            'sgbm_mode': 0,
            'prefilter_size': 9,
            'prefilter_cap': 31,
            'correlation_window_size': 61,
            'min_disparity': 0,
            'disparity_range': 192,
            'uniqueness_ratio': 5.0,
            'texture_threshold': 10,
            'speckle_size': 100,
            'speckle_range': 4,
        }],
        output='screen'
    )

    pointcloud_custom_node = Node(
        package='ros2_ps5_stereo',
        executable='disparity_to_pointcloud',
        name='disparity_to_pointcloud',
        output='screen',
        parameters=[{
            'roi_height': 2
        }]
    )

    point_cloud_node = Node(
        package='stereo_image_proc',
        executable='point_cloud_node',
        name='point_cloud_node',
        remappings=[
            ('left/image_rect_color', '/left/image_rect'),
        ],
        parameters=[{
            'queue_size': 10,
            'min_disparity': 0,
            'max_disparity': 192,
            'use_color': True
        }],
        output='screen',
        arguments=['--ros-args', '--log-level', 'error']
    )

    disparity_to_laserscan = Node(
        package='disparity_to_laserscan_cpp',
        executable='disparity_to_laserscan',
        name='disparity_to_laserscan',
        output='screen',
        parameters=[{
            'scan_height': 1,
            'range_min': 0.1,
            'range_max': 5.0
        }]
    )

    laserscan_filters = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter',
        parameters=[{
            'filter_chain_parameter_name': 'scan_filter_chain',
            'filter_chain_parameters_source': laser_filter_yaml
        }],
        # remappings=[]
    )

    staticTransformPointCloud = Node(            # Para el pointcloud node custom
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'base_link', 'lidar_link'],
    )

    staticTransformLaserscan = Node(              # Para el nodo de laserscan
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '3.14159', 'base_link', 'lidar_link'],
    )

    return LaunchDescription([
        camera_node,
        rectify_left,
        rectify_right,
        disparity_node,
        # pointcloud_custom_node,
        # point_cloud_node,
        # staticTransformPointCloud,
        disparity_to_laserscan,
        laserscan_filters,
        staticTransformLaserscan
    ])
