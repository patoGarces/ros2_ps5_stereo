from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Nodo de cámaras, ros2_ps5_stereo y el ejecutable camera_node
    camera_node = Node(
        package='ros2_ps5_stereo',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'camera_resolution': 1, #    RES_1920p = 0(OJO SIN CALIBRAR), RES_1080p = 1, RES_640p = 2(OJO SIN CALIBRAR)
            'camera_fps': 0,        #    FPS_8 = 0, FPS_30 = 1, FPS_60 = 2
            'roi_height': 50        #    crop de los frames de cameras NO SE UTILIZA ACTUALMENTE
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

    metric_depthmap = Node(
        package = 'depth_image_proc',
        executable = 'convert_metric_node',
        name = 'convert_metric_node',
        remappings=[
            ('disparity', '/disparity'),
            ('depth', '/depthmap1'),            # salida
            ('image_raw', '/right/image_rect')
        ],
        parameters=[{
            'min_depth': 0.1,   # mínimo 10 cm
            'max_depth': 5.0    # máximo 5 metros
        }],
        arguments=['--ros-args', '--log-level', 'debug']
    )

    staticTransformCameras = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'base_link', 'frame_left'],
    )

    return LaunchDescription([
        camera_node,
        rectify_left,
        rectify_right,
        disparity_node,
        pointcloud_custom_node,
        # point_cloud_node,
        # metric_depthmap,
        # pointcloud_from_depth,
        staticTransformCameras
    ])
