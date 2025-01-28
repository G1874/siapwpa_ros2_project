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
        
        # Set parameters for binarization, area filters, and image region cutoffs
        self.bin_thresh = bin_thresh
        self.area_max = area_max
        self.area_min = area_min
        self.hood_cutoff = hood_cutoff
        self.left_cutoff = left_cutoff
        self.right_cutoff = right_cutoff
        self.top_cutoff = top_cutoff
        
        # Subscription to the image topic from a camera feed
        self.subscription = self.create_subscription(
            Image,
            '/birdseye_view/image',  # Topic for input birdseye-view images
            self.image_callback,  # Callback function to process incoming images
            10
        )

        # Publisher for the binarized image
        self.publisher_ = self.create_publisher(Image, '/binarized_image', 10)
        
        # CvBridge to convert between ROS image messages and OpenCV images
        self.bridge = CvBridge()
        
        # Subscription to slider values for dynamic adjustment of the parameters
        self.slider_sub = self.create_subscription(
            Float32MultiArray,
            '/binarization_slider_values',  # Topic for slider values (to adjust thresholds and cutoffs)
            self.slider_callback,  # Callback function to update parameters based on slider input
            10  # Queue size for the subscription
        )

    def slider_callback(self, msg):
        """
        Callback to update binarization parameters based on slider input.
        """
        # Extract and set values from the incoming message (slider values)
        self.bin_thresh = int(msg.data[0])
        self.area_max = int(msg.data[1])
        self.area_min = int(msg.data[2])
        self.hood_cutoff = msg.data[3]
        self.left_cutoff = msg.data[4]
        self.right_cutoff = msg.data[5]
        self.top_cutoff = msg.data[6]
        
    def image_callback(self, msg):
        """
        Callback to process each incoming image.
        """
        try:
            # Convert ROS image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            # If conversion fails, log the error and return
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Convert the image from BGR to HSV color space (easier for color-based thresholding)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define the range for the yellow color in HSV space (for detecting road markings)
        lower_yellow = np.array([20, 100, 100], dtype=np.uint8)
        upper_yellow = np.array([30, 255, 255], dtype=np.uint8)
        
        # Create a binary mask where yellow pixels are white (1) and others are black (0)
        yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        x, y = yellow_mask.shape  # Get image dimensions

        # Find connected components (regions of white pixels)
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(yellow_mask, connectivity=8)

        # Create a mask to store only the components that meet area size criteria
        filtered_mask = np.zeros_like(yellow_mask)

        # Loop through each component and apply area filtering
        for label in range(1, num_labels):  # Ignore the background (label 0)
            area = stats[label, cv2.CC_STAT_AREA]
            if self.area_min <= area <= self.area_max:
                filtered_mask[labels == label] = 255  # Keep only components within the specified area range

        # Update yellow_mask with the filtered result
        yellow_mask = filtered_mask
        
        # Apply region cutoffs to remove areas outside of the desired region (e.g., road boundaries)
        yellow_mask[-int(x * self.hood_cutoff):, :] = 0  # Cut off the bottom region
        # The following cutoffs are commented out but can be used to remove the left, right, or top regions
        # yellow_mask[:, :int(y * self.left_cutoff)] = 0
        # yellow_mask[:, -int(y * self.right_cutoff):] = 0
        # yellow_mask[:int(x * self.top_cutoff), :] = 0

        try:
            # Convert the processed mask back to a ROS image message
            binarized_msg = self.bridge.cv2_to_imgmsg(yellow_mask, encoding='mono8')
            # Publish the binarized image
            self.publisher_.publish(binarized_msg)
        except Exception as e:
            # If conversion or publishing fails, log the error
            self.get_logger().error(f"Failed to publish image: {e}")


def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2 library
    print(args)
    # Create an instance of the ImageBinarizerNode with default parameters
    node = ImageBinarizerNode(55, area_max=234, area_min=142, hood_cutoff=0.01, left_cutoff=0.33, right_cutoff=0.5, top_cutoff=0.01)
    try:
        # Keep the node running until interrupted
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Gracefully handle Ctrl+C
    finally:
        node.destroy_node()  # Clean up the node when shutting down
        rclpy.shutdown()  # Shutdown ROS 2 system

if __name__ == '__main__':
    main()  # Entry point to run the program
