import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
import math

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        
        # Parameters
        self.lookahead_distance = self.declare_parameter('lookahead_distance', 1.0).value
        self.max_linear_speed = self.declare_parameter('max_linear_speed', 1.0).value
        self.max_angular_speed = self.declare_parameter('max_angular_speed', 1.0).value

        # Target point
        self.target_point = None

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.target_point_sub = self.create_subscription(Point, 'target_point', self.target_point_callback, 10)

    def target_point_callback(self, msg):
        # Update the target point from the topic message
        self.target_point = (msg.x, msg.y)
        self.get_logger().info(f"Target point set to ({msg.x}, {msg.y})")

    def odom_callback(self, msg):
        if self.target_point is None:
            return

        # Extract robot's current position and orientation
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(orientation)

        # Compute the distance and angle to the target point
        dx = self.target_point[0] - position.x
        dy = self.target_point[1] - position.y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - yaw)

        # Stop the robot if it has reached the target point
        if distance < self.lookahead_distance:
            self.publish_velocity(0.0, 0.0)
            return

        # Compute linear and angular velocities
        linear_speed = min(self.max_linear_speed, distance)
        angular_speed = min(self.max_angular_speed, angle_diff)

        # Publish the velocities
        self.publish_velocity(linear_speed, angular_speed)

    def publish_velocity(self, linear, angular):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular
        self.cmd_vel_pub.publish(cmd_vel)

    @staticmethod
    def quaternion_to_yaw(orientation):
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
