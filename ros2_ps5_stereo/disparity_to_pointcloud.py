import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, PointCloud2
from stereo_msgs.msg import DisparityImage

import numpy as np
import open3d as o3d

import cv2

from sensor_msgs_py.point_cloud2 import create_cloud_xyz32

class DisparityToPointCloudNode(Node):
    def __init__(self):
        super().__init__('disparity_to_pointcloud')
        self.sub_disp = self.create_subscription(
            DisparityImage,
            '/disparity',
            self.disparity_callback,
            10)
        self.sub_cam_info = self.create_subscription(
            CameraInfo,
            '/left/camera_info',
            self.cam_info_callback,
            10)
        self.pub_pc = self.create_publisher(PointCloud2, '/point_cloud_custom', 10)

        self.cam_info = None

    def cam_info_callback(self, msg: CameraInfo):
        self.cam_info = msg

    def disparity_callback(self, disp_msg: DisparityImage):
        if self.cam_info is None:
            self.get_logger().warn('No camera info received yet')
            return

        disp = np.frombuffer(disp_msg.image.data, dtype=np.float32).reshape(
            (disp_msg.image.height, disp_msg.image.width)
        )

        # Resize disparidad a 640x480 para acelerar
        disp_resized = cv2.resize(disp, (640, 480), interpolation=cv2.INTER_LINEAR)
        # disp_resized = disp

        # Parámetros cámara (puedes obtener más de cam_info si quieres)
        fx = self.cam_info.k[0]
        fy = self.cam_info.k[4]
        cx = self.cam_info.k[2]
        cy = self.cam_info.k[5]

        # Calcular nube de puntos
        points = []
        for v in range(disp.shape[0]):
            for u in range(disp_resized.shape[1]):
                d = disp[v, u]
                if d <= 0.0:
                    continue
                Z = fx * 0.05 / d  # baseline=0.05m (ajusta según tu cámara)
                X = (u - cx) * Z / fx
                Y = (v - cy) * Z / fy
                points.append([X, Y, Z])

        if not points:
            self.get_logger().warn('No valid points found in disparity')
            return

        points_np = np.array(points, dtype=np.float32)

        # Crear mensaje PointCloud2
        header = disp_msg.header
        pc_msg = create_cloud_xyz32(header, points_np)

        self.pub_pc.publish(pc_msg)
        self.get_logger().info(f'Published point cloud with {len(points)} points')

def main(args=None):
    rclpy.init(args=args)
    node = DisparityToPointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
