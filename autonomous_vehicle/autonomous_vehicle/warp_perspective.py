import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class PerspectiveWarpNode(Node):
    def __init__(self):
        super().__init__('warp_perspective_node')

        # Parameters
        self.declare_parameter('input_image_topic', '/front_camera_sensor/image')
        self.declare_parameter('output_image_topic', '/birdseye_view/image')

        # Load parameters
        self.input_image_topic = self.get_parameter('input_image_topic').value
        self.output_image_topic = self.get_parameter('output_image_topic').value

        # Initialize variables
        self.bridge = CvBridge()

        # Subscribers and publishers
        self.image_sub = self.create_subscription(Image, self.input_image_topic, self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, self.output_image_topic, 10)

        self.get_logger().info("Perspective warp node initialized.")

    def image_callback(self, msg):
        """ Callback to process the image and warp perspective. """
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Warp the perspective
            warped_image = self.update_perspective(cv_image)

            warped_msg = self.bridge.cv2_to_imgmsg(warped_image, encoding='bgr8')
            self.image_pub.publish(warped_msg)

            # self.get_logger().info("Published warped bird's-eye view image.")
        except Exception as e:
            self.get_logger().error(f"Error warping or publishing image: {e}")

    def update_perspective(self, img):
        w, h = (800, 800)

        img_pts = np.float32([
            [265,438],
            [92,519],
            [706,519],
            [533,438]
        ])

        model_pts = np.float32([
            [0,0],
            [0,6],
            [6,6],
            [6,0]
        ]) * 35 # 25 pixels per meter.

        model_pts[0:,0] += (w - 6*35) / 2
        model_pts[0:,1] += (h - 6*35 - 2*35)

        K = np.float32([
            [476.7030715942383, 0.0, 400.0],
            [0.0, 476.7030715942383, 400.0],
            [0.0, 0.0, 1.0]
        ])

        d = np.float32([0,0,0,0,0])

        img_undistorted = cv2.undistort(img, K, d)
        img_pts_undist = cv2.perspectiveTransform(cv2.undistortPoints(img_pts, K, d), K)

        H = cv2.getPerspectiveTransform(img_pts_undist, model_pts)

        img_warp = cv2.warpPerspective(
            img_undistorted,
            H, (w,h),
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0,0,0)
        )

        # for i in range(0, 4):
        #     cv2.circle(img_warp, (int(model_pts[i,0]), int(model_pts[i,1])), 3, (0,0,0), -1)

        # for i in range(0, 4):
        #     j = (i + 1)
        #     if i == 3 : j = 0
        #     cv2.line(img_warp, (int(model_pts[i,0]),int(model_pts[i,1])),
        #              (int(model_pts[j,0]),int(model_pts[j,1])), (0,0,0), 2)
        
        return img_warp


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