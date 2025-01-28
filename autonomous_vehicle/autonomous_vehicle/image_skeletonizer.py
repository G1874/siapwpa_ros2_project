import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray


class ImageSkeletonizerNode(Node):
    def __init__(self):
        super().__init__('road_skeletonizer')
        
        # Subscription to the binarized image topic
        self.subscription = self.create_subscription(
            Image,
            '/binarized_image',
            self.image_callback,
            10
        )

        # Publishers for road points and visualization images
        self.publisher_pts = self.create_publisher(Float32MultiArray, '/road_pts', 10)
        self.publisher_img = self.create_publisher(Image, '/road_viz', 10)

        # Initialize CvBridge to convert between ROS images and OpenCV images
        self.bridge = CvBridge()
        
    def image_callback(self, msg):
        # Convert the received ROS image message to an OpenCV image (grayscale)
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        
        # Find all non-zero points (white pixels) in the binary image
        points = cv2.findNonZero(cv_image)
        X, Y = cv_image.shape  # Get image dimensions

        if points is not None and len(points) > 1:  # Check if points are found and there are enough points for line fitting
            # Fit a straight line through the points using the least-squares method
            line_params = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
            vx, vy, x0, y0 = line_params.flatten()  # Extract line parameters (direction vector and a point on the line)

            # Calculate endpoints of the line to extend it across the entire image
            x1 = int(x0 - vx * X)
            y1 = int(y0 - vy * X)
            x2 = int(x0 + vx * X)
            y2 = int(y0 + vy * X)

            # Convert the grayscale image to a color image for visualization
            output_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

            # Draw the fitted line on the output image
            cv2.line(output_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        else:
            # If no points are found, log a warning and stop processing
            output_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            self.get_logger().warn("No points found for line fitting.")
            return
        
        # Generate evenly spaced points along the fitted line (20 points)
        pts = np.linspace([x1, y1], [x2, y2], 20)

        # Prepare the Float32MultiArray message for publishing the points
        points = Float32MultiArray()
        points_data = []

        for x, y in pts:
            # Mark the points on the output image as red circles
            output_image = cv2.circle(output_image, (int(x), int(y)), 10, (0, 0, 255), 2)
            points_data.append(x)  # Append x-coordinate
            points_data.append(y)  # Append y-coordinate

        # Set the points data in the message
        points.data = points_data

        # Publish the points data
        self.publisher_pts.publish(points)

        # Convert the processed image back to a ROS message
        processed_msg = self.bridge.cv2_to_imgmsg(output_image, encoding='bgr8')

        # Publish the visualization image
        self.publisher_img.publish(processed_msg)


def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python library
    node = ImageSkeletonizerNode()  # Create an instance of the node
    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully
    finally:
        node.destroy_node()  # Destroy the node when shutting down
        rclpy.shutdown()  # Shut down the ROS 2 system


if __name__ == '__main__':
    main()  # Entry point for the script
