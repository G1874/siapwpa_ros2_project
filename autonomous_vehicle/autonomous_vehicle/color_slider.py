import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

PI=3.14159265359

class SliderControlNode(Node):
    def __init__(self):
        super().__init__('slider_control_node')
        self.r = 0
        self.g = 0
        self.b = 0

        self.pub = self.create_publisher(Float32MultiArray, '/color_slider_values', 10)
        plt.subplots_adjust(left=0.1, bottom=0.1, top=0.9, right=0.9)

        self.slider_r = self.create_slider('r', 0.0, 255, self.r, 1.0, 0.85)
        self.slider_g = self.create_slider('g', 0.0, 255, self.g, 1.0, 0.75)
        self.slider_b = self.create_slider('b', 0.0, 255, self.b, 1.0, 0.65)

        self.get_logger().info("Slider control node initialized.")
        self.slider_update(0)

    def create_slider(self, label, min_val, max_val, init_val, val_step, pos_y):
        slider_ax = plt.axes([0.1, pos_y, 0.8, 0.05], facecolor='lightgoldenrodyellow')
        slider = Slider(slider_ax, label, min_val, max_val, valinit=init_val, valstep=val_step)
        slider.on_changed(self.slider_update)
        return slider

    def slider_update(self, val):
        self.r = int(self.slider_r.val)
        self.g = int(self.slider_g.val)
        self.b = int(self.slider_b.val)

        # self.get_logger().info(f"Updated values: bin_thresh={self.bin_thresh}, area_max={self.area_max}, area_min={self.area_min}, hood_cutoff={self.hood_cutoff}") 

        msg = Float32MultiArray()
        msg.data = list(map(float, [self.r, self.g, self.b]))
        self.pub.publish(msg)
        # self.get_logger().info("Published updated values to topic.")

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
