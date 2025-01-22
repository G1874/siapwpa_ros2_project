import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
from .stanley_controller.stanley_controller import *
from .stanley_controller import cubic_spline_planner
from matplotlib import pyplot as plt


class VehicleState(State):
    def __init__(self, dt):
        super().__init__()
        self.yaw_rate = 0.0
        self.dt = dt

    def update(self, v, yaw_rate):
        self.v = v
        self.yaw_rate = yaw_rate

        self.yaw += self.yaw_rate * self.dt
        self.yaw = normalize_angle(self.yaw)
        self.x += self.v * np.cos(self.yaw) * self.dt
        self.y += self.v * np.sin(self.yaw) * self.dt


class MotionControl(Node):
    def __init__(self):
        super().__init__('motion_control_node')

        # Parameters
        self.declare_parameter('coord_transform', '/birdseye_view/coord_transform')
        self.declare_parameter('cmd_vel', '/cmd_vel')
        self.declare_parameter('road_pts', '/road_pts')
        self.declare_parameter('odometry', '/odometry')
        self.declare_parameter('sign_detection', '/sign_detect')

        # Load parameters
        self.cmd_vel_test = self.get_parameter('cmd_vel').value
        self.coord_transform_topic = self.get_parameter('coord_transform').value
        self.road_pts_topic = self.get_parameter('road_pts').value
        self.odometry_topic = self.get_parameter('odometry').value
        self.sign_detection_class_topic = self.get_parameter('sign_detection').value
        
        # Initialize variables
        self.timer_1_period = 0.01    # Send setpoints to velocity controller.
        self.timer_2_period = 0.5      # Update trajectory.

        self.cmd_vel_msg = Twist()
        
        self.road_pts = None
        self.coord_transform = None
        self.sign_class = None
        self.current_speed = 1.0

        self.waypoints = None
        self.target_idx = None
        self.state = VehicleState(self.timer_1_period)

        self.H = None               # Coord transform matrix.

        # Timers
        self.timer_1 = self.create_timer(self.timer_1_period, self.timer_1_callback)
        self.timer_2 = self.create_timer(self.timer_2_period, self.timer_2_callback)

        # Subscribers and publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.coord_transform_sub = self.create_subscription(Float32MultiArray,
                                                            self.coord_transform_topic,
                                                            self.coord_transform_callback, 10)
        self.road_pts_sub = self.create_subscription(Float32MultiArray,
                                                     self.road_pts_topic,
                                                     self.road_pts_callback, 10)
        self.odometry_sub = self.create_subscription(Odometry,
                                                     self.odometry_topic,
                                                     self.update_state, 10)
        self.sign_class_sub = self.create_subscription(String,
                                                       self.sign_detection_class_topic,
                                                       self.sign_class_callback, 10)
        
    def sign_class_callback(self, msg):
        if "Speed limit" in msg.data:
            speed = float(msg.data.split('(')[1].split('km/h')[0].strip())
            self.current_speed = float(speed) / 10.0
        self.get_logger().info(f"Detected sign: {msg.data}, setting speed to {self.current_speed} km/h")

    def timer_1_callback(self):
        self.get_logger().info(f"{self.current_speed}")
        self.motion_controller(self.waypoints, self.current_speed)

    def timer_2_callback(self):
        if self.road_pts is not None:
            pts = np.concatenate((self.road_pts, np.ones((self.road_pts.shape[0],1))), axis=1)            
            pts = np.array([np.matmul(self.H, v.T).T for v in pts])

            x = list(pts[0:,0])
            y = list(pts[0:,1])

            self.get_logger().info(f'x: {x}, y: {y}')

            c_x, c_y, c_yaw, _, _ = cubic_spline_planner.calc_spline_course(x, y, ds=0.1)
            
            self.waypoints = (c_x, c_y, c_yaw)
            self.target_idx, _ = calc_target_index(self.state, c_x, c_y)

            self.state.x = 0.0
            self.state.y = 0.0
            self.state.yaw = 0.0

    def coord_transform_callback(self, msg):
        if self.coord_transform is None:  # Read it only once.
            self.coord_transform = msg.data
            self.H = np.array(self.coord_transform).reshape((3,3))

    def road_pts_callback(self, msg):
        points = np.array(msg.data, dtype=np.float32)
        self.road_pts = np.reshape(points, (points.size//2, 2))

    def update_state(self, msg):
        self.state.v = msg.twist.twist.linear.x
        self.state.yaw_rate = msg.twist.twist.angular.z

        # pos_x = msg.pose.position.x
        # pos_y = msg.pose.position.y

        self.display()

    def send_setpoints(self, target_vel, target_yaw_rate):
        self.cmd_vel_msg.linear.x = target_vel
        self.cmd_vel_msg.angular.z = target_yaw_rate

        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def motion_controller(self, waypoints=None, target_vel=0.0):
        if waypoints is not None:
            c_x = waypoints[0]
            c_y = waypoints[1]
            c_yaw = waypoints[2]

            delta, self.target_idx = stanley_control(self.state, c_x, c_y, c_yaw, self.target_idx)
            turningRadius = L * np.tan(delta)
            # turningRadius = L / np.sin(delta)
            target_yaw_rate = self.state.v / turningRadius

            self.send_setpoints(target_vel, target_yaw_rate)
        else:
            self.send_setpoints(target_vel, 0.0)

    def display(self, pos_x=None, pos_y=None):
        # plt.cla()
        
        # if self.waypoints is not None:
        #     c_x = self.waypoints[0]
        #     c_y = self.waypoints[1]
        #     plt.plot(c_x, c_y, "-r", label="course")
        #     plt.plot(c_x[self.target_idx], c_y[self.target_idx], "xg", label="target")
        
        # if self.road_pts is not None:
        #     H = self.coord_transform
        #     H = np.array(H).reshape((3,3))
        #     pts = np.concatenate((self.road_pts, np.ones((self.road_pts.shape[0],1))), axis=1)            
        #     pts = np.array([np.matmul(v.T, H).T for v in pts])
        #     plt.plot(pts[0:,0], pts[0:,1], ".k", label="course")

        plt.plot(self.state.x, self.state.y, "-b", label="trajectory")

        # plt.axis("equal")
        # plt.grid(True)

    


def main(args=None):
    rclpy.init(args=args)
    node = MotionControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()