import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray

class ImageBinarizerNode(Node):
    def __init__(self, bin_thresh, area_max, area_min, hood_cutoff, left_cutoff, right_cutoff):
        super().__init__('image_binarizer')
        self.bin_thresh = bin_thresh
        self.area_max = area_max
        self.area_min = area_min
        self.hood_cutoff = hood_cutoff
        ##
        self.left_cutoff = left_cutoff
        self.right_cutoff = right_cutoff
        ##
        self.subscription = self.create_subscription(
            Image,
            '/birdseye_view/image',
            self.image_callback,
            10
        )

        self.publisher_ = self.create_publisher(Image, '/binarized_image', 10)
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
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, binarized_image = cv2.threshold(gray_image, self.bin_thresh, 255, cv2.THRESH_BINARY)
        x, y = binarized_image.shape
    

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binarized_image, connectivity=8)

        filtered_mask = np.zeros_like(binarized_image)
        for label in range(1, num_labels): 
            area = stats[label, cv2.CC_STAT_AREA]
            if self.area_min <= area <= self.area_max:
                filtered_mask[labels == label] = 255
        binarized_image = filtered_mask

        binarized_image[-int(x*self.hood_cutoff):, :] = 0
        binarized_image[:, :int(y*self.left_cutoff)] = 0
        binarized_image[:, -int(y*self.right_cutoff):] = 0
        # # self.get_logger().info(f"{-int(y*self.left_cutoff)}")
        # self.get_logger().info(f"{self.left_cutoff}")

        # coordinates = np.column_stack(np.where(binarized_image == 255))
        # if coordinates.size > 0:
        #     lowest_point = coordinates[np.argmax(coordinates[:, 0])]
        #     highest_point = coordinates[np.argmin(coordinates[:, 0])]
        # binarized_image = cv2.cvtColor(binarized_image, cv2.COLOR_GRAY2BGR)
        # binarized_image = cv2.drawMarker(binarized_image, tuple(lowest_point), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
        # binarized_image = cv2.drawMarker(binarized_image, tuple(highest_point), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

        # binarized_image[labels != max_label] = 0

        try:
            # binarized_msg = self.bridge.cv2_to_imgmsg(binarized_image, encoding='bgr8')
            binarized_msg = self.bridge.cv2_to_imgmsg(binarized_image, encoding='mono8')
            self.publisher_.publish(binarized_msg)
            pass
            # self.get_logger().info("Published the binarized image")
        except Exception as e:
            pass
            # self.get_logger().error(f"Failed to convert or publish image: {e}")

import sys
def main(args=None):
    # thresh = int(sys.argv[1]) if len(sys.argv) > 1 else 127
    # area = int(sys.argv[2]) if len(sys.argv) > 2 else 800
    # area_min = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    rclpy.init(args=args)
    print(args)
    node = ImageBinarizerNode(60, area_max=5000, area_min=0, hood_cutoff=0.06, left_cutoff=0.26, right_cutoff=0.6)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
