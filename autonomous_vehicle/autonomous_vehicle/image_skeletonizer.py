import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped

class ImageSkeletonizerNode(Node):
    def __init__(self):
        super().__init__('road_skeletonizer')
        self.subscription = self.create_subscription(
            Image,
            '/binarized_image',
            self.image_callback,
            10
        )

        self.publisher_pts = self.create_publisher(Float32MultiArray, '/road_pts', 10)
        self.publisher_img = self.create_publisher(Image, '/road_viz', 10)
        self.publisher_pt = self.create_publisher(PointStamped, '/road_target', 10)
        self.bridge = CvBridge()
        

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        points = cv2.findNonZero(cv_image)

        X, Y = cv_image.shape
        if points is not None and len(points) > 1:
            self.get_logger().info(f"Found {points} points.")
            point_to_return = points[np.argmax(points[:, :, 0]), :, :].astype(float)
            lowest_point = points[np.argmax(points[:, :, 0]), :, :].astype(float)
            points = np.vstack([point_to_return, lowest_point])
            line_params = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
            vx, vy, x0, y0 = line_params.flatten()
            x1 = int(x0 - vx * X)
            y1 = int(y0 - vy * X)
            x2 = int(x0 + vx * X)
            y2 = int(y0 + vy * X)
            output_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            cv2.line(output_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        else:
            output_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            self.get_logger().warn("No points found for line fitting.")
            return
        
        pts = np.linspace([x1, y1], [x2, y2], 20)
        points = Float32MultiArray()
        points_data = []
        for x, y in pts:
            output_image = cv2.circle(output_image, (int(x), int(y)), 10, (0, 0, 255), 2)
            points_data.append(x)
            points_data.append(y)
        points.data = points_data
        # na przemian x, y
        self.publisher_pts.publish(points)
        output_image = cv2.circle(output_image, (int(point_to_return[0, 0]), int(point_to_return[0, 1])), 10, (255, 0, 0), 2)
        ########
        one_pt = PointStamped()
        one_pt.point.x = float(pts[15, 0])
        one_pt.point.y = float(pts[15, 1])
        # one_pt.point.x = point_to_return[0, 0]
        # one_pt.point.y = point_to_return[0, 1]
        self.publisher_pt.publish(one_pt)
        ########
        processed_msg = self.bridge.cv2_to_imgmsg(output_image, encoding='bgr8')
        

        self.publisher_img.publish(processed_msg)

    # def image_callback(self, msg):
    #     cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
    #     points = cv2.findNonZero(cv_image)

    #     if points is not None and len(points) > 1:
    #         point_to_return = points[np.argmax(points[:, :, 0]), :, :].astype(float)
    #     else:
    #         return
        
    #     one_pt = PointStamped()
    #     X = point_to_return[0, 0] - 100
    #     Y = point_to_return[0, 1]
    #     self.publisher_pt.publish(one_pt)

    #     output_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
    #     output_image = cv2.circle(output_image, (int(X), int(Y)), 10, (255, 0, 0), 2)
    #     processed_msg = self.bridge.cv2_to_imgmsg(output_image, encoding='bgr8')
    #     self.publisher_img.publish(processed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSkeletonizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
