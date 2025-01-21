import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np
from .stanley_controller.stanley_controller import *
from .stanley_controller import cubic_spline_planner


class VehicleState(State):
    def __init__(self):
        super().__init__()

    def update(self, v, yaw, dt):
        self.v = v
        self.yaw = yaw
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt


class MotionControl(Node):
    def __init__(self):
        super().__init__('motion_control_node')

        # Parameters
        self.declare_parameter('coord_transform', '/birdseye_view/coord_transform')
        self.declare_parameter('cmd_vel', '/cmd_vel')
        self.declare_parameter('road_pts', '/road_pts')

        # Load parameters
        self.cmd_vel_test = self.get_parameter('cmd_vel').value
        self.coord_transform_topic = self.get_parameter('coord_transform').value
        self.road_pts_topic = self.get_parameter('road_pts').value

        # Initialize variables
        self.timer_1_period = 0.1    # Send setpoints to velocity controller.
        self.timer_2_period = 2      # Get new trajectory.

        self.cmd_vel_msg = Twist()
        
        self.road_pts = None
        self.coord_transform = None

        self.waypoints = None
        self.target_idx = None
        self.state = VehicleState()

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

    def timer_1_callback(self):
        self.motion_controller(self.waypoints, 20.0)

    def timer_2_callback(self):
        if type(self.road_pts) == None:
            x = list(self.road_pts[0:,0]) # TODO: Change coordinates of points, consider downsampling.
            y = list(self.road_pts[0:,1])

            c_x, c_y, c_yaw, _, _ = cubic_spline_planner.calc_spline_course(x, y, ds=0.1)
            
            self.waypoints = (c_x, c_y, c_yaw)
            self.target_idx, _ = calc_target_index(self.state, c_x, c_y)

    def coord_transform_callback(self, msg):
        if type(self.coord_transform) == None:  # Read it only once.
            self.coord_transform = msg.data
        
    def road_pts_callback(self, msg):
        points = np.array(msg.data, dtype=np.float32)
        self.road_pts = np.reshape(points, (points.size//2, 2))

    def send_setpoints(self, forward, yaw):
        self.cmd_vel_msg.linear.x = forward
        self.cmd_vel_msg.angular.z = yaw

        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def motion_controller(self, waypoints, vel_setpoint):
        c_x = waypoints[0]
        c_y = waypoints[1]
        c_yaw = waypoints[2]

        # TODO: Figure it out. 
        delta, self.target_idx = stanley_control(self.state, c_x, c_y, c_yaw, self.target_idx)        

        self.send_setpoints(vel_setpoint, 5)

    def update_state(self):
        # TODO:
        pass


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