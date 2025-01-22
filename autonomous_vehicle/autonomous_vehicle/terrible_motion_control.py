import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        
        # Parameters
        self.declare_parameter('lookahead_distance', 3.0)
        self.declare_parameter('wheelbase', 2.5)
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.wheelbase = self.get_parameter('wheelbase').value
        
        # Subscribers and Publishers
        self.subscription_target = self.create_subscription(
            PointStamped,
            '/road_target',
            self.target_callback,
            10
        )

        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odometry',
            self.odom_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variables to store current state
        self.current_position = None
        self.current_heading = None
        self.target_position = None

        self.get_logger().info("Pure Pursuit controller node has been started.")

    def odom_callback(self, msg):
        """Callback for odometry data, updates current position and heading."""
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        self.current_heading = yaw

    def target_callback(self, msg):
        """Callback for target point."""
        self.target_position = (msg.point.x, msg.point.y)
        self.compute_control()

    def compute_control(self):
        """Compute the steering and velocity commands based on Pure Pursuit."""
        if self.current_position is None or self.current_heading is None or self.target_position is None:
            self.get_logger().warn("Waiting for odometry and target point data...")
            return

        # Calculate lookahead distance
        dx = self.target_position[0] - self.current_position[0]
        dy = self.target_position[1] - self.current_position[1]
        distance_to_target = np.hypot(dx, dy)

        # Check if target is within lookahead distance
        if distance_to_target < self.lookahead_distance:
            self.get_logger().info("Target within lookahead distance. No steering needed.")
            return

        # Calculate angle to target
        angle_to_target = np.arctan2(dy, dx)
        heading_error = angle_to_target - self.current_heading

        # Normalize angle to [-pi, pi]
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        # Pure Pursuit steering calculation
        curvature = 2 * np.sin(heading_error) / self.lookahead_distance
        steering_angle = np.arctan(self.wheelbase * curvature)

        # Publish velocity command
        twist_msg = Twist()
        twist_msg.linear.x = 8.0  # Constant speed (can be tuned)
        twist_msg.angular.z = steering_angle  # Steering command

        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info(f"Published cmd_vel: linear={twist_msg.linear.x}, angular={twist_msg.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
