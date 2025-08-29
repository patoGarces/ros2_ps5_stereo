import rclpy
from rclpy.node import Node
from stereo_msgs.msg import DisparityImage

class DisparityRelayNode(Node):
    def __init__(self):
        super().__init__('disparity_relay_qos')
        self.subscription = self.create_subscription(
            DisparityImage,
            '/disparity',
            self.listener_callback,
            qos_profile=rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                depth=10
            )
        )
        self.publisher = self.create_publisher(
            DisparityImage,
            '/disparity_reliable',
            qos_profile=rclpy.qos.QoSProfile(
                # reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,

                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                depth=10
            )
        )

    def listener_callback(self, msg):
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DisparityRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
