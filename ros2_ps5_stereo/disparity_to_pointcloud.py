import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, PointCloud2
from stereo_msgs.msg import DisparityImage

import numpy as np
import open3d as o3d

from sensor_msgs_py.point_cloud2 import create_cloud_xyz32

class DisparityToPointCloudNode(Node):
    def __init__(self):
        super().__init__('disparity_to_pointcloud')

        self.declare_parameter('roi_height', 25)                                        # píxeles arriba/abajo del centro
        # Leer parámetros del nodo
        self.roi_height = self.get_parameter('roi_height').value

        self.get_logger().info(f'Roi recibido: {self.roi_height}')

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
        # disp_resized = cv2.resize(disp, (640, 480), interpolation=cv2.INTER_LINEAR)
        disp_resized = disp

        h_center = disp_resized.shape[0] // 2
        roi_top = max(0, h_center - self.roi_height)
        roi_bottom = min(disp_resized.shape[0], h_center + self.roi_height)
        disp_roi = disp_resized[roi_top:roi_bottom, :]


        # Parámetros cámara
        fx = self.cam_info.k[0]
        fy = self.cam_info.k[4]
        cx_full = self.cam_info.k[2]
        cy_full = self.cam_info.k[5]

        # --- Recalcular 'cy' relativo a la ROI
        # cy_roi es el índice del centro óptico dentro del array recortado
        cy_roi = cy_full - roi_top

        # --- Baseline --
        baseline = 0.05  # metros

        # --- Reproyección sobre la ROI --
        points = []
        roi_h, roi_w = disp_roi.shape
        for v_roi in range(roi_h):                # fila dentro de la ROI
            for u in range(roi_w):                # columna (no recortamos horizontalmente)
                d = disp_roi[v_roi, u]
                if not np.isfinite(d) or d <= 0.0:
                    continue

                # Z = baseline * fx / disparity
                Z = baseline * fx / d

                # u corresponde a la columna en la imagen completa si no recortaste horizontalmente.
                # Si recortaste horizontalmente, usa u_img = u + roi_left.
                u_img = u

                # X y Y usando el centro óptico y la ROI correctamente ajustada para Y
                X = (u_img - cx_full) * Z / fx
                Y = (v_roi - cy_roi) * Z / fy   # <-- cy_roi: centro relativo a la ROI

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
