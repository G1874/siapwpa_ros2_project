# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# from std_msgs.msg import Float32MultiArray

# class ImageBinarizerNode(Node):
#     def __init__(self, bin_thresh, area_max, area_min, hood_cutoff, left_cutoff, right_cutoff, top_cutoff):
#         super().__init__('image_binarizer')
#         self.bin_thresh = bin_thresh
#         self.area_max = area_max
#         self.area_min = area_min
#         self.hood_cutoff = hood_cutoff
#         ##
#         self.left_cutoff = left_cutoff
#         self.right_cutoff = right_cutoff
#         self.top_cutoff = top_cutoff
#         ##
#         self.subscription = self.create_subscription(
#             Image,
#             '/birdseye_view/image',
#             self.image_callback,
#             10
#         )

#         self.publisher_ = self.create_publisher(Image, '/binarized_image', 10)
#         self.bridge = CvBridge()
#         self.slider_sub = self.create_subscription(Float32MultiArray, '/binarization_slider_values', self.slider_callback, 10)


#     def slider_callback(self, msg):
#         self.bin_thresh = int(msg.data[0])
#         self.area_max = int(msg.data[1])
#         self.area_min = int(msg.data[2])
#         self.hood_cutoff = msg.data[3]
#         self.left_cutoff = msg.data[4]
#         self.right_cutoff = msg.data[5]
#         self.top_cutoff = msg.data[6]
        

#     def image_callback(self, msg):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f"Failed to convert image: {e}")
#             return

#         gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
#         _, binarized_image = cv2.threshold(gray_image, self.bin_thresh, 255, cv2.THRESH_BINARY)
#         x, y = binarized_image.shape
    

#         num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binarized_image, connectivity=8)

#         filtered_mask = np.zeros_like(binarized_image)
#         for label in range(1, num_labels): 
#             area = stats[label, cv2.CC_STAT_AREA]
#             if self.area_min <= area <= self.area_max:
#                 filtered_mask[labels == label] = 255
#         binarized_image = filtered_mask

#         binarized_image[-int(x*self.hood_cutoff):, :] = 0
#         binarized_image[:, :int(y*self.left_cutoff)] = 0
#         binarized_image[:, -int(y*self.right_cutoff):] = 0
#         binarized_image[:int(x*self.top_cutoff), :] = 0
#         try:
#             binarized_msg = self.bridge.cv2_to_imgmsg(binarized_image, encoding='mono8')
#             self.publisher_.publish(binarized_msg)
#         except Exception as e:
#             pass

# import sys
# def main(args=None):
#     rclpy.init(args=args)
#     print(args)
#     node = ImageBinarizerNode(55, area_max=481, area_min=142, hood_cutoff=0.01, left_cutoff=0.33, right_cutoff=0.6, top_cutoff=0.01)
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray

class ImageBinarizerNode(Node):
    def __init__(self, bin_thresh, area_max, area_min, hood_cutoff, left_cutoff, right_cutoff, top_cutoff):
        super().__init__('image_binarizer')
        self.bin_thresh = bin_thresh
        self.area_max = area_max
        self.area_min = area_min
        self.hood_cutoff = hood_cutoff
        ##
        self.left_cutoff = left_cutoff
        self.right_cutoff = right_cutoff
        self.top_cutoff = top_cutoff
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
        self.top_cutoff = msg.data[6]
        

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100], dtype=np.uint8)
        upper_yellow = np.array([30, 255, 255], dtype=np.uint8)
        yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        x, y = yellow_mask.shape

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(yellow_mask, connectivity=8)

        filtered_mask = np.zeros_like(yellow_mask)
        for label in range(1, num_labels): 
            area = stats[label, cv2.CC_STAT_AREA]
            if self.area_min <= area <= self.area_max:
                filtered_mask[labels == label] = 255

        yellow_mask = filtered_mask
        
        yellow_mask[-int(x * self.hood_cutoff):, :] = 0
        # yellow_mask[:, :int(y * self.left_cutoff)] = 0
        # yellow_mask[:, -int(y * self.right_cutoff):] = 0
        # yellow_mask[:int(x * self.top_cutoff), :] = 0

        try:
            binarized_msg = self.bridge.cv2_to_imgmsg(yellow_mask, encoding='mono8')
            self.publisher_.publish(binarized_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")



import sys
def main(args=None):
    rclpy.init(args=args)
    print(args)
    node = ImageBinarizerNode(55, area_max=234, area_min=142, hood_cutoff=0.01, left_cutoff=0.33, right_cutoff=0.5, top_cutoff=0.01)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
