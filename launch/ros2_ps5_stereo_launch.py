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

    # Nodo de dispariity
    # disparity_node = Node(
    #     package='stereo_image_proc',
    #     executable='disparity_node',
    #     name='disparity_node',
    #     parameters=[{
    #         'min_disparity': 0,
    #         'max_disparity': 128,  # múltiplo de 16
    #         # 'block_size': 100,
    #         'correlation_window_size': 128,
    #         'uniqueness_ratio': 5.0,
    #     }]
    # )

    disparity_node = Node(
        package='stereo_image_proc',
        executable='disparity_node',
        name='disparity_node',
        parameters=[{
            # Algoritmo SGBM
            'sgbm_mode': 0,

            # Disparity pre-filtering
            'prefilter_size': 9,         # ventana de normalización (pixeles)
            'prefilter_cap': 31,         # límite en valores normalizados

            # Disparity correlation
            'correlation_window_size': 61,  # tamaño de la ventana para correlación (debe ser impar, 5-255)
            'min_disparity': 0,           # disparidad mínima (offset de búsqueda)
            'disparity_range': 192,       # tamaño del rango de disparidad (pixeles)

            # Post-filtering
            'uniqueness_ratio': 5.0,     # filtrado por razón de unicidad
            'texture_threshold': 10,     # filtro basado en textura mínima
            'speckle_size': 100,         # tamaño mínimo de regiones para aceptar disparidad
            'speckle_range': 4,          # rango para agrupar regiones conectadas
        }]
    )

    # point_cloud_node = Node(
    #     package='stereo_image_proc',
    #     executable='point_cloud_node',
    #     name='point_cloud_node',
    #     # remappings=remappings,
    # )

    point_cloud_node = Node(
        package='stereo_image_proc',
        executable='point_cloud_node',
        name='point_cloud_node',
        remappings=[
            # ('left/image_rect', '/left/image_rect'),
            # ('right/image_rect', '/right/image_rect'),
            # ('left/camera_info', '/left/camera_info'),
            # ('right/camera_info', '/right/camera_info')
            ('left/image_rect_color', '/left/image_rect'),
            ('/disparity', '/disparity'),
            ('left/camera_info', '/left/camera_info'),
            ('right/camera_info', '/right/camera_info')
        ],
        output='screen'
    )

    metric_depthmap = Node(
        package = 'depth_image_proc',
        executable = 'convert_metric_node',
        name = 'convert_metric_node',
        remappings=[
            ('disparity', '/disparity'),
            ('depth', '/depthmap1')  # Salida
        ]
    )


    return LaunchDescription([
        camera_node,
        rectify_left,
        rectify_right,
        disparity_node,
        point_cloud_node,
        metric_depthmap
    ])
