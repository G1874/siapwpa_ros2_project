import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

PI=3.14159265359

class SliderControlNode(Node):
    def __init__(self):
        super().__init__('slider_control_node')
        self.alpha = 4.838
        self.beta = 0.0
        self.gamma = 0.0
        self.f = 200.0
        self.dist = 456
        self.pub = self.create_publisher(Float32MultiArray, '/birdseye_slider_values', 10)
        plt.subplots_adjust(left=0.1, bottom=0.1, top=0.9, right=0.9)

        self.slider_alpha = self.create_slider(r'$\alpha$', 0.0, 2*PI, self.alpha, 0.001, 0.85)
        self.slider_beta = self.create_slider(r'$\beta$', 0.0, 2*PI, self.beta, 0.001, 0.75)
        self.slider_gamma = self.create_slider(r'$\gamma$', 0.0, 2*PI, self.gamma, 0.001, 0.65)
        self.slider_focal = self.create_slider('f', 0.0, 2000.0, self.f, 0.1, 0.55)
        self.slider_dist = self.create_slider('d', 0.0, 2000.0, self.dist, 0.1, 0.45)

        self.get_logger().info("Slider control node initialized.")
        self.slider_update(0)

    def create_slider(self, label, min_val, max_val, init_val, val_step, pos_y):
        slider_ax = plt.axes([0.1, pos_y, 0.8, 0.05], facecolor='lightgoldenrodyellow')
        slider = Slider(slider_ax, label, min_val, max_val, valinit=init_val, valstep=val_step)
        slider.on_changed(self.slider_update)
        return slider

    def slider_update(self, val):
        self.alpha = self.slider_alpha.val
        self.beta = self.slider_beta.val
        self.gamma = self.slider_gamma.val
        self.f = self.slider_focal.val
        self.dist = self.slider_dist.val

        self.get_logger().info(f"Updated values: Alpha={self.alpha:.2f}, Beta={self.beta:.2f}, "
                               f"Gamma={self.gamma:.2f}, f={self.f:.2f}, Dist={self.dist:.2f}")

        msg = Float32MultiArray()
        msg.data = [self.alpha, self.beta, self.gamma, self.f, self.dist]
        self.pub.publish(msg)
        self.get_logger().info("Published updated values to topic.")

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = SliderControlNode()
    plt.show()

    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
