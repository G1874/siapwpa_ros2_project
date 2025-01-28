import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
from .stanley_controller.stanley_controller import *
from .stanley_controller import cubic_spline_planner


class VehicleState(State):
    ''' Class for storing current vehicle state. '''

    def __init__(self, dt):
        super().__init__()
        self.yaw_rate = 0.0
        self.dt = dt

    def update(self, v, yaw_rate):
        self.v = v
        self.yaw_rate = yaw_rate

        # Current position is estimated based on velocity measurement.
        self.yaw += self.yaw_rate * self.dt
        self.yaw = normalize_angle(self.yaw)
        self.x += self.v * np.cos(self.yaw) * self.dt
        self.y += self.v * np.sin(self.yaw) * self.dt


class MotionControl(Node):
    ''' Update the state of the vehicle based on velocity and yaw rate. '''

    def __init__(self):
        super().__init__('motion_control_node')

        # Parameters
        self.declare_parameter('coord_transform', '/birdseye_view/coord_transform')  # Declaring parameter for coordinate transform topic
        self.declare_parameter('cmd_vel', '/cmd_vel')  # Declaring parameter for cmd_vel topic
        self.declare_parameter('road_pts', '/road_pts')  # Declaring parameter for road points topic
        self.declare_parameter('odometry', '/odometry')  # Declaring parameter for odometry topic
        self.declare_parameter('sign_detection', '/sign_detect')  # Declaring parameter for sign detection topic

        # Load parameters
        self.cmd_vel_test = self.get_parameter('cmd_vel').value  # Retrieve cmd_vel topic
        self.coord_transform_topic = self.get_parameter('coord_transform').value  # Retrieve coord_transform topic
        self.road_pts_topic = self.get_parameter('road_pts').value  # Retrieve road_pts topic
        self.odometry_topic = self.get_parameter('odometry').value  # Retrieve odometry topic
        self.sign_detection_class_topic = self.get_parameter('sign_detection').value  # Retrieve sign_detection topic
        
        # Initialize variables
        self.timer_1_period = 0.01      # Send setpoints to velocity controller.
        self.timer_2_period = 0.5       # Update trajectory.

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
        self.timer_1 = self.create_timer(self.timer_1_period, self.timer_1_callback)  # Timer for velocity setpoints
        self.timer_2 = self.create_timer(self.timer_2_period, self.timer_2_callback)  # Timer for trajectory update

        # Subscribers and publishers.
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)  # Publisher for cmd_vel topic
        self.coord_transform_sub = self.create_subscription(Float32MultiArray,
                                                            self.coord_transform_topic,
                                                            self.coord_transform_callback, 10)  # Subscriber for coord transform
        self.road_pts_sub = self.create_subscription(Float32MultiArray,
                                                     self.road_pts_topic,
                                                     self.road_pts_callback, 10)  # Subscriber for road points
        self.odometry_sub = self.create_subscription(Odometry,
                                                     self.odometry_topic,
                                                     self.update_state, 10)  # Subscriber for odometry
        self.sign_class_sub = self.create_subscription(String,
                                                       self.sign_detection_class_topic,
                                                       self.sign_class_callback, 10)  # Subscriber for sign detection class
        
    def sign_class_callback(self, msg):
        ''' Method called after a sign detection, changes velocity setpoint
            based on the speed limit. '''

        if "Speed limit" in msg.data:
            speed = float(msg.data.split('(')[1].split('km/h')[0].strip())  # Extract speed from message
            self.current_speed = float(speed) / 10.0  # Convert speed limit to target velocity
        self.get_logger().info(f"Detected sign: {msg.data}, setting speed to {self.current_speed} km/h")

    def timer_1_callback(self):
        ''' Callback for sending new velocity setpoints (period of 0.01s). '''

        self.motion_controller(self.waypoints, self.current_speed)  # Update velocity setpoints

    def timer_2_callback(self):
        ''' Callback for creating a new spline trajectory (period of 0.5s). '''

        if self.road_pts is not None:
            # Points of the detected lines are converted to the coordinate
            # system of the vehicle.
            pts = np.concatenate((self.road_pts, np.ones((self.road_pts.shape[0], 1))), axis=1)  # Convert road points to homogeneous coordinates
            pts = np.array([np.matmul(self.H, v.T).T for v in pts])  # Apply coordinate transformation

            x = list(pts[0:,0])
            y = list(pts[0:,1])

            # Spline trajectory is computed from points of the detected lines.
            c_x, c_y, c_yaw, _, _ = cubic_spline_planner.calc_spline_course(x, y, ds=0.1)
            
            self.waypoints = (c_x, c_y, c_yaw)  # Store new waypoints
            # Index of the closest point on the trajectory.
            self.target_idx, _ = calc_target_index(self.state, c_x, c_y)

            # Since a new trajectory is computed estimated positions are set back to zero.
            self.state.x = 0.0
            self.state.y = 0.0
            self.state.yaw = 0.0

    def coord_transform_callback(self, msg):
        ''' Method for receiving the coordinate transformation matrix from warp_perspective node. '''

        if self.coord_transform is None:  # Read it only once.
            self.coord_transform = msg.data
            self.H = np.array(self.coord_transform).reshape((3, 3))  # Convert to transformation matrix

    def road_pts_callback(self, msg):
        ''' Method for receiving points of the detected lines from image_skeletonizer node. '''

        points = np.array(msg.data, dtype=np.float32)  # Convert message data to numpy array
        self.road_pts = np.reshape(points, (points.size // 2, 2))  # Reshape the points into (x, y) pairs

    def update_state(self, msg):
        ''' Update vehicle velocities after receiving the measurement from odometry topic. '''

        self.state.v = msg.twist.twist.linear.x  # Update vehicle velocity
        self.state.yaw_rate = msg.twist.twist.angular.z  # Update vehicle yaw rate

    def send_setpoints(self, target_vel, target_yaw_rate):
        ''' Send velocity setpoints to the ackerman controller. '''

        self.cmd_vel_msg.linear.x = target_vel  # Set target linear velocity
        self.cmd_vel_msg.angular.z = target_yaw_rate  # Set target angular velocity

        self.cmd_vel_pub.publish(self.cmd_vel_msg)  # Publish the velocity setpoint

    def motion_controller(self, waypoints=None, target_vel=0.0):
        ''' Calculate and send new setpoints. '''

        if waypoints is not None:
            c_x = waypoints[0]
            c_y = waypoints[1]
            c_yaw = waypoints[2]

            # Calculate target steering angle delta.
            delta, self.target_idx = stanley_control(self.state, c_x, c_y, c_yaw, self.target_idx)  # Stanley control
            # Convert to target yaw rate.
            turningRadius = L / np.sin(delta)  # Calculate turning radius
            target_yaw_rate = self.state.v / turningRadius  # Calculate target yaw rate

            self.send_setpoints(target_vel, target_yaw_rate)  # Send setpoints

            # Convert for readibility and display through console.
            delta_conv = np.sign(delta)*(90 - np.rad2deg(np.abs(delta)))  # Convert delta for display
            self.get_logger().info(f"v: {target_vel}, delta: {delta_conv}")
        else:
            target_yaw_rate = 0.0  # Default target yaw rate
            self.send_setpoints(target_vel, target_yaw_rate)  # Send default setpoints


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
