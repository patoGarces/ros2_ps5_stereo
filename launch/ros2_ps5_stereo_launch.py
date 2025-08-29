from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Nodo de cámaras, ros2_ps5_stereo y el ejecutable camera_node
    camera_node = Node(
        package='ros2_ps5_stereo',
        executable='camera_node',
        name='camera_node',
        output='screen',
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

    relay_node = Node(
        package='ros2_ps5_stereo',
        executable='relay_node_disparity',  # el script que compilaste
        name='disparity_relay_qos',
        output='screen'
    )

    disparity_to_pointcloud_node = Node(
        package='ros2_ps5_stereo',
        executable='disparity_to_pointcloud',
        name='disparity_to_pointcloud',
        output='screen',
    )


    point_cloud_node = Node(
        package='stereo_image_proc',
        executable='point_cloud_node',
        name='point_cloud_node',
        remappings=[
            ('left/image_rect', '/left/image_rect'),
            ('right/image_rect', '/right/image_rect'),
            # ('disparity_image', '/disparity'),            // TODO: cambiar el disparity_image por /disparity directametne

            ('disparity', '/disparity_reliable'),


            ('left/camera_info', '/left/camera_info'),
            ('right/camera_info', '/right/camera_info'),
            ('left/image_rect_color', '/left/image_rect'),
        ],
        output='screen'
    )


    # metric_depthmap = Node(
    #     package = 'depth_image_proc',
    #     executable = 'convert_metric_node',
    #     name = 'convert_metric_node',
    #     remappings=[
    #         ('disparity', '/disparity'),
    #         ('depth', '/depthmap1')  # Salida
    #     ]
    # )

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


        # relay_node,   // TODO: no sirvio, limpiar file y setup.py
        disparity_to_pointcloud_node,

        point_cloud_node,
        # metric_depthmap,
        staticTransformCameras
    ])
