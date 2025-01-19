import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np
from stanley_controller.stanley_controller import *
from stanley_controller.PathPlanning.CubicSpline import cubic_spline_planner


class MotionControl(Node):
    def __init__(self):
        super().__init__('motion_control_node')

        # Parameters
        self.declare_parameter('coord_transform', '/birdseye_view/coord_transform')
        self.declare_parameter('cmd_vel', '/cmd_vel')
        self.declare_parameter('road_pts', '/road_pts')

        # Load parameters
        self.cmd_vel_test = self.declare_parameter('cmd_vel').value
        self.coord_transform_topic = self.get_parameter('coord_transform').value
        self.road_pts_topic = self.get_parameter('road_pts').value

        # Initialize variables
        timer_1_period = 0.1    # Send setpoints to velocity controller.
        timer_2_period = 2      # Get new trajectory.

        self.cmd_vel_msg = Twist()

        # Timers
        self.timer_1 = self.create_timer(timer_1_period, self.timer_1_callback)
        self.timer_2 = self.create_timer(timer_2_period, self.timer_2_callback)

        # Subscribers and publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.coord_transform_sub = self.create_subscription(Float32MultiArray,
                                                            self.coord_transform_topic, 10)
        self.road_pts_sub = self.create_subscription(Float32MultiArray, self.road_pts_topic, 10)

        def timer_1_callback(self):
            pass

        def timer_2_callback(self):
            pass

        def send_setpoints(self, forward, yaw):
            self.cmd_vel_msg.linear.x = forward
            self.cmd_vel_msg.angular.z = yaw

            self.publisher_.cmd_vel_pub(self.cmd_vel_msg)

        def motion_controller(self, waypoints):
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