import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo
import yaml
import os

from copy import deepcopy
from ros2_ps5_stereo.cameraPs5Handler import CameraPs5Handler
from ros2_ps5_stereo.utilsClass import Resolutions, Utils

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.logger = self.get_logger()

        # Parámetros del nodo
        self.declare_parameter('camera_resolution', Resolutions.RES_1280x800_8FPS.value)    # entero que mapea al enum de resolution
        self.declare_parameter('roi_height', 25)                                            # píxeles arriba/abajo del centro

        # Leer parámetros
        res_enum_param = Resolutions(self.get_parameter('camera_resolution').value)
        roi_height_param = self.get_parameter('roi_height').value

        self.logger.info(
            f'Resolución seleccionada: {res_enum_param}, ROI: {roi_height_param} pixeles'
        )

        self.bridge = CvBridge()

        # Publishers para cada frame (izquierda y derecha)
        self.pub_left_image = self.create_publisher(Image, 'left/image_raw', 10)
        self.pub_right_image = self.create_publisher(Image, 'right/image_raw', 10)

        # Publishers para cada info (izquierda y derecha)
        self.pub_left_info = self.create_publisher(CameraInfo, 'left/camera_info', 10)
        self.pub_right_info = self.create_publisher(CameraInfo, 'right/camera_info', 10)

        # Servicios para actualizar la calibracion
        self.left_set_info_srv = self.create_service(
            SetCameraInfo, 'left_camera/set_camera_info', self.left_set_camera_info_callback)
        self.right_set_info_srv = self.create_service(
            SetCameraInfo, 'right_camera/set_camera_info', self.right_set_camera_info_callback)

        calibLeftPath, calibRightPath = self.__getCalibYamlPath(res_enum_param)
        calibLeft = self.__getCameraInfo(calibLeftPath)
        calibRight = self.__getCameraInfo(calibRightPath)

        self.left_camera_info = self.__convertYamlToCameraInfo(calibLeft)
        self.left_camera_info.header.frame_id = 'frame_left'
        self.right_camera_info = self.__convertYamlToCameraInfo(calibRight)
        self.right_camera_info.header.frame_id = 'frame_right'

        self.cameraPs5Handler = CameraPs5Handler(logger = self.logger, resolution_enum=res_enum_param, roi_height=roi_height_param)
        self.cameraPs5Handler.setCbFrames(self.framePublisher)

        # TODO: pendiente almacenar los yaml recibidos en el service de SetCameraInfo
        # TODO: al almacenarlos se debe remplazar el camera name

    def framePublisher(self, frameL,frameR):
        # Convertir OpenCV a Image ROS
        msg_left_image = self.bridge.cv2_to_imgmsg(frameL, encoding='bgr8')
        msg_right_image = self.bridge.cv2_to_imgmsg(frameR, encoding='bgr8')

        timestamp = self.get_clock().now().to_msg()
        msg_left_image.header.frame_id = 'frame_left'
        msg_left_image.header.stamp = timestamp
        msg_right_image.header.frame_id = 'frame_right'
        msg_right_image.header.stamp = timestamp

        self.left_camera_info.header.stamp = timestamp
        self.right_camera_info.header.stamp = timestamp

        # Publicar
        self.pub_left_image.publish(msg_left_image)
        self.pub_right_image.publish(msg_right_image)
        self.pub_left_info.publish(self.left_camera_info)
        self.pub_right_info.publish(self.right_camera_info)

    def left_set_camera_info_callback(self, request, response):
        self.get_logger().info('Received left camera calibration update')
        self.left_camera_info = deepcopy(request.camera_info)
        self.left_camera_info.header.frame_id = 'frame_left'
        response.success = True
        response.status_message = 'Left camera calibration updated'
        return response

    def right_set_camera_info_callback(self, request, response):
        self.get_logger().info('Received right camera calibration update')
        self.right_camera_info = deepcopy(request.camera_info)
        self.right_camera_info.header.frame_id = 'frame_right'
        response.success = True
        response.status_message = 'Right camera calibration updated'
        return response
    
    def __convertYamlToCameraInfo(self, calib_dict: dict) -> CameraInfo:
        msg = CameraInfo()
        msg.width = calib_dict['image_width']
        msg.height = calib_dict['image_height']
        msg.distortion_model = calib_dict['distortion_model']
        msg.d = calib_dict['distortion_coefficients']['data']
        msg.k = calib_dict['camera_matrix']['data']
        msg.r = calib_dict['rectification_matrix']['data']
        msg.p = calib_dict['projection_matrix']['data']
        return msg
    
    def __getCameraInfo(self,yaml_path):
        with open(yaml_path, 'r') as file:
            calib_data = yaml.safe_load(file)
        return calib_data
    
    def __getCalibYamlPath(self, resolution: Resolutions):
        base_path = 'src/ros2_ps5_stereo/ros2_ps5_stereo/calibration/'

        mapping = {
            Resolutions.RES_640x480_DOWNSAMPLED_8FPS:   "640x480",
            Resolutions.RES_640x480_DOWNSAMPLED_30FPS:  "640x480",
            Resolutions.RES_640x480_DOWNSAMPLED_60FPS:  "640x480",
            Resolutions.RES_1280x800_8FPS:              "1280x800",
            Resolutions.RES_1280x800_30FPS:             "1280x800",
            Resolutions.RES_1280x800_60FPS:             "1280x800",
            Resolutions.RES_1920x1080_8FPS:             "1920x1080",
            Resolutions.RES_1920x1080_30FPS:            "1920x1080",
        }

        folder = mapping[resolution]

        calib_left  = os.path.join(base_path, folder, "left.yaml")
        calib_right = os.path.join(base_path, folder, "right.yaml")

        return calib_left, calib_right

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
