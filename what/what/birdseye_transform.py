import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class PerspectiveWarpNode(Node):
    def __init__(self):
        super().__init__('perspective_warp_node')

        # Parameters
        self.declare_parameter('input_image_topic', '/front_camera_sensor/image')
        self.declare_parameter('output_image_topic', '/birdseye_view/image')
        self.declare_parameter('output_size', [800, 800])

        self.alpha = 0
        self.beta = 0
        self.gamma = 0  
        self.focalLength = 1
        self.dist = 1

        # Load parameters
        self.input_image_topic = self.get_parameter('input_image_topic').value
        self.output_image_topic = self.get_parameter('output_image_topic').value
        self.output_size = tuple(self.get_parameter('output_size').value)

        # Initialize variables
        self.bridge = CvBridge()

        # Subscribers and publishers
        self.image_sub = self.create_subscription(
            Image, self.input_image_topic, self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, self.output_image_topic, 10)
        self.slider_sub = self.create_subscription(Float32MultiArray, '/birdseye_slider_values', self.slider_callback, 10)

        # self.get_logger().info("Perspective warp node initialized.")

    def slider_callback(self, msg):
        print(msg.data)
        self.alpha = msg.data[0]
        self.beta = msg.data[1]
        self.gamma = msg.data[2]
        self.focalLength = msg.data[3]
        self.dist = msg.data[4]

    def image_callback(self, msg):
        
        """Callback to process the image and warp perspective."""

        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Warp the perspective
            warped_image = self.update_perspective(cv_image)

            warped_msg = self.bridge.cv2_to_imgmsg(warped_image, encoding='bgr8')
            self.image_pub.publish(warped_msg)
            pass
            # self.get_logger().info("Published warped bird's-eye view image.")
        except Exception as e:
            pass
            # self.get_logger().error(f"Error warping or publishing image: {e}")


    def update_perspective(self, source):
        alpha = self.alpha
        beta = self.beta
        gamma = self.gamma
        dist = self.dist
        focalLength = self.focalLength

        w, h = (800, 800)

        A1 = np.array([[1, 0, -w / 2],
                    [0, 1, -h / 2],
                    [0, 0, 0],
                    [0, 0, 1]], dtype=np.float32)

        RX = np.array([[1, 0, 0, 0],
                    [0, math.cos(alpha), -math.sin(alpha), 0],
                    [0, math.sin(alpha), math.cos(alpha), 0],
                    [0, 0, 0, 1]], dtype=np.float32)

        RY = np.array([[math.cos(beta), 0, -math.sin(beta), 0],
                    [0, 1, 0, 0],
                    [math.sin(beta), 0, math.cos(beta), 0],
                    [0, 0, 0, 1]], dtype=np.float32)

        RZ = np.array([[math.cos(gamma), -math.sin(gamma), 0, 0],
                    [math.sin(gamma), math.cos(gamma), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]], dtype=np.float32)

        R = np.dot(np.dot(RX, RY), RZ)

        T = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, dist],
                    [0, 0, 0, 1]], dtype=np.float32)

        K = np.array([[focalLength, 0, w / 2, 0],
                    [0, focalLength, h / 2, 0],
                    [0, 0, 1, 0]], dtype=np.float32)

        transformationMat = np.dot(np.dot(np.dot(K, T), R), A1)

        destination = cv2.warpPerspective(source, transformationMat, (w, h), flags=cv2.INTER_CUBIC + cv2.WARP_INVERSE_MAP)
        return destination

def main(args=None):
    rclpy.init(args=args)
    node = PerspectiveWarpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

      
