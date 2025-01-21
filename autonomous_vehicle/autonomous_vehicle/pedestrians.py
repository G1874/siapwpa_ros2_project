# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2
# from sensor_msgs_py import point_cloud2
# import matplotlib.pyplot as plt
# import numpy as np
# import cv2

# class PointCloudSubscriber(Node):
#     def __init__(self):
#         super().__init__('pointcloud_subscriber')
        
#         # Subscribe to the PointCloud2 topic
#         self.subscription = self.create_subscription(
#             PointCloud2,
#             '/center_laser_sensor/scan_points',
#             self.point_cloud_callback,
#             10  # QoS profile
#         )
#         self.get_logger().info("PointCloud2 subscriber node started.")
#         self.fig = plt.figure()
#         self.distance_threshold = 20

#     def generate_front_image(self, points):
#         Y, Z = points[:, points[0, :] > 0][1:3, :]
#         self.get_logger().info(f"Y: {Y.max}, Z: {Z.max}")

        
#         Y = ((Y/self.distance_threshold + self.distance_threshold // 2) * 799).astype(int)
#         Z = (Z/self.distance_threshold * 599).astype(int)
        
#         front_image = np.zeros((600, 800))
#         front_image[Z, Y] = 255
#         cv2.imshow('Front Image', front_image)
#         cv2.waitKey(1)

    
#     def visualize_point_cloud(self, points):
#         cond = np.sum(np.power(points[:2, :], 2), axis=0)<=self.distance_threshold**2
#         cond2 = np.logical_and(points[2, :] > -6, points[2, :] < 0)
#         # pointsR = points[:, np.logical_and(cond, cond2)]
#         # pointsB = points[:, np.logical_and(np.logical_not(cond), np.logical_not(cond2))]
#         pointsR = points[:, cond2]
#         pointsB = points[:, np.logical_not(cond2)]
#         XB, YB, ZB = pointsB
#         XR, YR, ZR = pointsR
#         ax = self.fig.add_subplot(projection='3d')
#         ax.scatter(XB, YB, ZB, c='b', marker='.', label='Blue')
#         ax.scatter(XR, YR, ZR, c='r', marker='.', label='Red')
#         # Set axis labels
#         ax.set_xlabel("X")
#         ax.set_ylabel("Y")
#         ax.set_zlabel("Z")
#         ax.set_title("Real-time Point Cloud Visualization")
#         plt.show()
#         plt.draw()  # Redraw the updated plot
#         plt.pause(0.001)  # Pause to allow updating without blocking
        
#     def point_cloud_to_array(self, points):
#         pts = [tuple(pt) for pt in points]
#         return np.array(pts).T

#     def fitler_point_array(self, points):
#         # cutoff cylinder
#         points = points[:, np.sum(np.power(points[:2], 2), axis=0)<=self.distance_threshold**2]
#         points = points[:, np.logical_and(points[2, :] > -6, points[2, :] < 0)]
#         return points

    
#     def point_cloud_callback(self, msg):
#         # Convert PointCloud2 to list of points
#         pc_data = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
#         points = self.point_cloud_to_array(pc_data)
#         points = self.fitler_point_array(points)
#         # self.visualize_point_cloud(points)
#         self.generate_front_image(points)

# def main(args=None):
#     rclpy.init(args=args)
    
#     pointcloud_subscriber = PointCloudSubscriber()

#     # Spin to keep the node running
#     rclpy.spin(pointcloud_subscriber)

#     # D

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import matplotlib.pyplot as plt
import numpy as np
import cv2

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')
        
        # Subscribe to the PointCloud2 topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/center_laser_sensor/scan_points',
            self.point_cloud_callback,
            10  # QoS profile
        )
        self.get_logger().info("PointCloud2 subscriber node started.")
        self.fig = plt.figure()
        self.distance_threshold = 20

    def generate_front_image(self, points):
        Y, Z = points[:, points[0, :] > 0][1:3, :]

        self.get_logger().info(f"Y max: {np.max(Y)}, Z max: {np.max(Z)}")
        
        # Normalize coordinates safely
        Y = np.clip(((Y / self.distance_threshold + 0.5) * 799), 0, 799).astype(int)
        Z = np.clip((Z / self.distance_threshold * 599), 0, 599).astype(int)
        
        # Ensure the points fit within the image size
        front_image = np.zeros((600, 800), dtype=np.uint8)
        
        # Assign intensity only for valid points
        try:
            front_image[Z, Y] = 255
        except IndexError as e:
            self.get_logger().warn(f"Indexing error in image generation: {e}")
        
        cv2.imshow('Front Image', front_image)
        cv2.waitKey(1)


    
    def visualize_point_cloud(self, points):
        cond = np.sum(np.power(points[:2, :], 2), axis=0)<=self.distance_threshold**2
        cond2 = np.logical_and(points[2, :] > -6, points[2, :] < 0)
        # pointsR = points[:, np.logical_and(cond, cond2)]
        # pointsB = points[:, np.logical_and(np.logical_not(cond), np.logical_not(cond2))]
        pointsR = points[:, cond2]
        pointsB = points[:, np.logical_not(cond2)]
        XB, YB, ZB = pointsB
        XR, YR, ZR = pointsR
        ax = self.fig.add_subplot(projection='3d')
        ax.scatter(XB, YB, ZB, c='b', marker='.', label='Blue')
        ax.scatter(XR, YR, ZR, c='r', marker='.', label='Red')
        # Set axis labels
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("Real-time Point Cloud Visualization")
        plt.show()
        plt.draw()  # Redraw the updated plot
        plt.pause(0.001)  # Pause to allow updating without blocking
        
    def point_cloud_to_array(self, points):
        pts = [tuple(pt) for pt in points]
        return np.array(pts).T

    def filter_point_array(self, points):
        # Filter points within cylinder radius and height
        mask = (np.sum(points[:2, :] ** 2, axis=0) <= self.distance_threshold ** 2) & \
            (points[2, :] > -6) & (points[2, :] < 0)
        
        return points[:, mask]


    
    def point_cloud_callback(self, msg):
        # Convert PointCloud2 to list of points
        pc_data = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = self.point_cloud_to_array(pc_data)
        points = self.filter_point_array(points)
        # self.visualize_point_cloud(points)
        self.generate_front_image(points)

def main(args=None):
    rclpy.init(args=args)
    
    pointcloud_subscriber = PointCloudSubscriber()

    # Spin to keep the node running
    rclpy.spin(pointcloud_subscriber)

    # D

if __name__ == '__main__':
    main()