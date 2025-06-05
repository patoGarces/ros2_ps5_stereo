import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo
import yaml

from copy import deepcopy

import queue
from ros2_ps5_stereo.cameraPs5Handler import CameraPs5Handler

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
       
        self.cameraPs5Handler = CameraPs5Handler()
        self.frame_queue = self.cameraPs5Handler.getQueueFrames()
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

        calibLeft = self.__getCameraInfo('src/ros2_ps5_stereo/ros2_ps5_stereo/calibration/left.yaml')
        calibRight = self.__getCameraInfo('src/ros2_ps5_stereo/ros2_ps5_stereo/calibration/right.yaml')

        self.left_camera_info = self.__convertYamlToCameraInfo(calibLeft)
        self.left_camera_info.header.frame_id = 'frame_left'
        self.right_camera_info = self.__convertYamlToCameraInfo(calibRight)
        self.right_camera_info.header.frame_id = 'frame_right'

        print('calib left: ' + str(self.left_camera_info))
        print('calib right: ' + str(self.right_camera_info))

        # TODO: pendiente almacenar los yaml recibidos en el service de SetCameraInfo
        # TODO: al almacenarlos se debe remplazar el camera name

        self.timer = self.create_timer(0.033, self.framePublisher)  # ~30Hz


    def framePublisher(self):
        try:
            frames = self.frame_queue.get_nowait()  # sin bloqueo
        except queue.Empty:
            return  # no hay frames, salimos

        frameR, frameL = frames

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



def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
