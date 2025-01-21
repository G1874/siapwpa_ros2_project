import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray
from skimage.morphology import skeletonize

class ImageBinarizerNode(Node):
    def __init__(self, bin_thresh, area_max, area_min, hood_cutoff, left_cutoff, right_cutoff):
        super().__init__('image_binarizer')
        self.bin_thresh = bin_thresh
        self.area_max = area_max
        self.area_min = area_min
        self.hood_cutoff = hood_cutoff
        self.left_cutoff = left_cutoff
        self.right_cutoff = right_cutoff
        self.subscription = self.create_subscription(
            Image,
            '/front_camera_sensor/image',
            self.image_callback,
            10
        )

        self.publisher_ = self.create_publisher(Image, '/binarized_road_image', 10)
        self.bridge = CvBridge()
        self.slider_sub = self.create_subscription(Float32MultiArray, '/binarization_slider_values', self.slider_callback, 10)


    def slider_callback(self, msg):
        self.bin_thresh = int(msg.data[0])
        self.area_max = int(msg.data[1])
        self.area_min = int(msg.data[2])
        self.hood_cutoff = msg.data[3]
        self.left_cutoff = msg.data[4]
        self.right_cutoff = msg.data[5]
        

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            x, y = cv_image.shape[0:2]
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        ##################
        # Temp
        # lt = self.left_cutoff
        # ht = self.right_cutoff
        # bot_cutoff = self.hood_cutoff
        # top_cutoff = self.left_cutoff
        ##################

        lt = 0.12
        ht = 0.22
        bot_cutoff = 0.5
        top_cutoff = 0.32
        
        low_thresh = (np.ones((3,))*lt*255).astype(np.uint8)
        high_thresh = (np.ones((3,))*ht*255).astype(np.uint8)
        binarized_image = cv2.inRange(cv_image, low_thresh, high_thresh)
        binarized_image[0: int(x*bot_cutoff), :] = 0
        binarized_image[int(x*(1-top_cutoff)):, :] = 0

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binarized_image, connectivity=8)
        largest_area_idx = np.argmax(stats[1:, cv2.CC_STAT_AREA])
        # binarized_image = np.where(labels == largest_area_idx+1, 255, 0).astype(np.uint8)
        binarized_image = labels == largest_area_idx+1
        # binarized_image = skeletonize(binarized_image)
        cv2.imshow('binarized_image', (binarized_image).astype(np.uint8)*255)
        cv2.waitKey(1)
        
        # binarized_msg = self.bridge.cv2_to_imgmsg(binarized_image, encoding='mono8')
        # self.publisher_.publish(binarized_msg)

import sys
def main(args=None):
    rclpy.init(args=args)
    print(args)
    node = ImageBinarizerNode(bin_thresh=60, area_max=5000, area_min=0, hood_cutoff=0.06, left_cutoff=0.26, right_cutoff=0.6)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
