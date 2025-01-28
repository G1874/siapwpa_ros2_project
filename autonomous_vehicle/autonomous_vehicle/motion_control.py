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


# VehicleState class manages the vehicle's state and updates its position and orientation
class VehicleState(State):
    def __init__(self, dt):
        super().__init__()
        self.yaw_rate = 0.0  # Initial yaw rate
        self.dt = dt  # Time step for updates

    # Updates the vehicle's position and orientation based on velocity and yaw rate
    def update(self, v, yaw_rate):
        self.v = v
        self.yaw_rate = yaw_rate

        self.yaw += self.yaw_rate * self.dt  # Update yaw angle
        self.yaw = normalize_angle(self.yaw)  # Normalize yaw to [-π, π]
        self.x += self.v * np.cos(self.yaw) * self.dt  # Update x-coordinate
        self.y += self.v * np.sin(self.yaw) * self.dt  # Update y-coordinate


# MotionControl class implements the main logic for motion control using Stanley Controller
class MotionControl(Node):
    def __init__(self):
        super().__init__('motion_control_node')

        # Declare parameters to configure topics
        self.declare_parameter('coord_transform', '/birdseye_view/coord_transform')
        self.declare_parameter('cmd_vel', '/cmd_vel')
        self.declare_parameter('road_pts', '/road_pts')
        self.declare_parameter('odometry', '/odometry')
        self.declare_parameter('sign_detection', '/sign_detect')

        # Load parameter values
        self.cmd_vel_test = self.get_parameter('cmd_vel').value
        self.coord_transform_topic = self.get_parameter('coord_transform').value
        self.road_pts_topic = self.get_parameter('road_pts').value
        self.odometry_topic = self.get_parameter('odometry').value
        self.sign_detection_class_topic = self.get_parameter('sign_detection').value

        # Initialize timing periods for the timers
        self.timer_1_period = 0.01  # Period for sending velocity setpoints
        self.timer_2_period = 0.5   # Period for updating the trajectory

        self.cmd_vel_msg = Twist()  # Initialize velocity message
        
        # Initialize variables
        self.road_pts = None  # Road points received from the camera
        self.coord_transform = None  # Coordinate transformation matrix
        self.sign_class = None  # Detected traffic sign class
        self.current_speed = 1.0  # Default vehicle speed

        self.waypoints = None  # Waypoints for path planning
        self.target_idx = None  # Current target index on the path
        self.state = VehicleState(self.timer_1_period)  # Initialize vehicle state

        self.H = None  # Homogeneous transformation matrix for coordinates

        # Create timers for periodic callbacks
        self.timer_1 = self.create_timer(self.timer_1_period, self.timer_1_callback)
        self.timer_2 = self.create_timer(self.timer_2_period, self.timer_2_callback)

        # Create publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)  # Publish velocity commands
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
        
    # Callback for handling detected traffic signs and adjusting speed
    def sign_class_callback(self, msg):
        if "Speed limit" in msg.data:
            speed = float(msg.data.split('(')[1].split('km/h')[0].strip())
            self.current_speed = float(speed) / 10.0  # Convert speed to internal units
        self.get_logger().info(f"Detected sign: {msg.data}, setting speed to {self.current_speed} km/h")

    # Timer callback for controlling the vehicle's motion
    def timer_1_callback(self):
        self.motion_controller(self.waypoints, self.current_speed)

    # Timer callback for updating the trajectory based on road points
    def timer_2_callback(self):
        if self.road_pts is not None:
            # Transform road points using the coordinate transformation matrix
            pts = np.concatenate((self.road_pts, np.ones((self.road_pts.shape[0], 1))), axis=1)            
            pts = np.array([np.matmul(self.H, v.T).T for v in pts])

            x = list(pts[0:, 0])  # Extract x-coordinates
            y = list(pts[0:, 1])  # Extract y-coordinates

            # Generate waypoints using cubic spline interpolation
            c_x, c_y, c_yaw, _, _ = cubic_spline_planner.calc_spline_course(x, y, ds=0.1)
            
            self.waypoints = (c_x, c_y, c_yaw)
            self.target_idx, _ = calc_target_index(self.state, c_x, c_y)

            # Reset vehicle state for new trajectory
            self.state.x = 0.0
            self.state.y = 0.0
            self.state.yaw = 0.0

    # Callback for receiving the coordinate transformation matrix
    def coord_transform_callback(self, msg):
        if self.coord_transform is None:  # Read the transformation matrix only once
            self.coord_transform = msg.data
            self.H = np.array(self.coord_transform).reshape((3, 3))

    # Callback for receiving road points detected by the camera
    def road_pts_callback(self, msg):
        points = np.array(msg.data, dtype=np.float32)
        self.road_pts = np.reshape(points, (points.size // 2, 2))

    # Update the vehicle's state based on odometry data
    def update_state(self, msg):
        self.state.v = msg.twist.twist.linear.x  # Update velocity
        self.state.yaw_rate = msg.twist.twist.angular.z  # Update yaw rate
        self.display()

    # Send velocity and yaw rate setpoints to the vehicle
    def send_setpoints(self, target_vel, target_yaw_rate):
        self.cmd_vel_msg.linear.x = target_vel
        self.cmd_vel_msg.angular.z = target_yaw_rate

        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    # Main motion controller using Stanley method
    def motion_controller(self, waypoints=None, target_vel=0.0):
        if waypoints is not None:
            c_x = waypoints[0]  # x-coordinates of the waypoints
            c_y = waypoints[1]  # y-coordinates of the waypoints
            c_yaw = waypoints[2]  # yaw angles of the waypoints

            # Compute the steering angle (delta) and update the target index
            delta, self.target_idx = stanley_control(self.state, c_x, c_y, c_yaw, self.target_idx)
            
            # Calculate turning radius from the steering angle
            turningRadius = L / np.sin(delta)
            target_yaw_rate = self.state.v / turningRadius

            # Send setpoints to the vehicle
            self.send_setpoints(target_vel, target_yaw_rate)

            # Log the steering angle for debugging
            delta_conv = np.sign(delta) * (90 - np.rad2deg(np.abs(delta)))
            self.get_logger().info(f"v: {target_vel}, delta: {delta_conv}")
        else:
            # Stop the vehicle if no waypoints are provided
            target_yaw_rate = 0.0
            self.send_setpoints(target_vel, target_yaw_rate)


# Main function to initialize and run the node
def main(args=None):
    rclpy.init(args=args)
    node = MotionControl()

    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # Clean up the node
        rclpy.shutdown()  # Shut down the ROS client library

# Entry point for the script
if __name__ == '__main__':
    main()
