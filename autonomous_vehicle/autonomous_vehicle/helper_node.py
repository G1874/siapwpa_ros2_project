import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


def find_green_marker(img):
    green_tresh_hi = np.array([50, 255, 50])
    green_tresh_lo = np.array([0, 100, 0])

    green_mask = cv2.inRange(img, green_tresh_lo, green_tresh_hi)
    img_green = cv2.bitwise_and(img, img, mask=green_mask)

    img_green = cv2.cvtColor(img_green, cv2.COLOR_BGR2GRAY)
    _, img_green = cv2.threshold(img_green, 1, 255, cv2.THRESH_BINARY)

    marker_y, marker_x = [np.average(indices) for indices in np.where(img_green >= 255)]

    return int(marker_x), int(marker_y)


def find_red_markers(img):
    red_tresh_hi = np.array([50, 50, 255])
    red_tresh_lo = np.array([0, 0, 100])

    red_mask = cv2.inRange(img, red_tresh_lo, red_tresh_hi)
    img_red = cv2.bitwise_and(img, img, mask=red_mask)

    img_red = cv2.cvtColor(img_red, cv2.COLOR_BGR2GRAY)
    _, img_red = cv2.threshold(img_red, 1, 255, cv2.THRESH_BINARY)

    img_quadrant = np.zeros((img_red.shape[0], img_red.shape[1], 4))
    img_quadrant[:(img_red.shape[0] // 2 + 100), :(img_red.shape[1] // 2), 0] = 1
    img_quadrant[(img_red.shape[0] // 2 + 100):, :(img_red.shape[1] // 2), 1] = 1
    img_quadrant[(img_red.shape[0] // 2 + 100):, (img_red.shape[1] // 2):, 2] = 1
    img_quadrant[:(img_red.shape[0] // 2 + 100), (img_red.shape[1] // 2):, 3] = 1

    markers =  np.zeros((4, 2), dtype=int)
    for i in range(0, 4):
        img_maker = img_red.copy()
        img_maker[img_quadrant[0:,0:,i] != 1] = 0

        marker_y, marker_x = [np.average(indices) for indices in np.where(img_maker >= 255)]
        markers[i,0:] = [int(marker_x), int(marker_y)]

    return markers


class HelperNode(Node):
    def __init__(self):
        super().__init__('helper_node')

        # Parameters
        self.declare_parameter('input_image_topic', '/front_camera_sensor/image')
        self.declare_parameter('birdseye_image_topic', '/birdseye_view/image')
        self.declare_parameter('road_viz', '/road_viz')

        # Load parameters
        self.input_image_topic = self.get_parameter('input_image_topic').value
        self.birdseye_image_topic = self.get_parameter('birdseye_image_topic').value
        self.road_viz_topic = self.get_parameter('road_viz').value
        
        # Initialize variables
        self.bridge = CvBridge()

        self.birdseye_image = np.zeros((800, 800))

        # Subscribers and publishers
        self.image_sub = self.create_subscription(Image, self.input_image_topic, self.display_image, 10)
        self.birdseye_sub = self.create_subscription(Image, self.birdseye_image_topic, self.display_image_birdseye, 10)
        self.road_viz_sub = self.create_subscription(Image, self.road_viz_topic, self.display_road_vis, 10)

    def proces_image(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        w, h = (800, 800)

        # img_pts = find_red_markers(img).astype(np.float32)
        img_pts = np.float32([
            [265,438],
            [92,519],
            [706,519],
            [533,438]
        ])

        # model_pts = np.float32([
        #     [300,500],
        #     [300,700],
        #     [500,700],
        #     [500,500]
        # ])

        model_pts = np.float32([
            [0,0],
            [0,6],
            [6,6],
            [6,0]
        ]) * 35 # 35 pixels per meter.

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

        for i in range(0, 4):
            cv2.circle(img_warp, (int(model_pts[i,0]), int(model_pts[i,1])), 3, (0,0,0), -1)

        for i in range(0, 4):
            j = (i + 1)
            if i == 3 : j = 0
            cv2.line(img_warp, (int(model_pts[i,0]),int(model_pts[i,1])),
                     (int(model_pts[j,0]),int(model_pts[j,1])), (0,0,0), 2)

        print(model_pts)

        cv2.imshow("img", img)
        cv2.imshow("img warp", img_warp)
        cv2.waitKey(1)

    def display_image_birdseye(self, msg):
        self.birdseye_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # for i in range(0, 4):
        #     cv2.circle(cv_image, (int(model_pts[i,0]), int(model_pts[i,1])), 3, (0,0,0), -1)

        # for i in range(0, 4):
        #     j = (i + 1)
        #     if i == 3 : j = 0
        #     cv2.line(cv_image, (int(model_pts[i,0]),int(model_pts[i,1])),
        #              (int(model_pts[j,0]),int(model_pts[j,1])), (0,0,0), 2)

        cv2.imshow("img warp", self.birdseye_image)
        cv2.waitKey(1)

    def display_image(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        cv2.imshow("img 2", cv_image)
        cv2.waitKey(1)

    def display_road_vis(self, msg):
        road_vis_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        disp_image = self.birdseye_image
        disp_image[road_vis_image != 0] = 0

        cv2.imshow("road_vis", disp_image)
        cv2.waitKey(1)
        

def main(args=None):
    rclpy.init(args=args)
    node = HelperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()