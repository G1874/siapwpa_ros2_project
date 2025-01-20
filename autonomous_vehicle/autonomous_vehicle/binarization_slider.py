import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

PI=3.14159265359

class SliderControlNode(Node):
    def __init__(self):
        super().__init__('slider_control_node')
        self.bin_thresh = 55.0
        self.area_max = 481
        self.area_min = 142
        self.hood_cutoff = 0.01
        self.left_cutoff = 0.33
        self.right_cutoff = 0.6
        self.top_cutoff = 0.01
        self.pub = self.create_publisher(Float32MultiArray, '/binarization_slider_values', 10)
        plt.subplots_adjust(left=0.1, bottom=0.1, top=0.9, right=0.9)

        self.slider_bin_thresh = self.create_slider('bin_thresh', 0.0, 255.0, self.bin_thresh, 1.0, 0.85)
        self.slider_area_max = self.create_slider('max_area', 0.0, 30000.0, self.area_max, 1.0, 0.75)
        self.slider_area_min = self.create_slider('min_area', 0.0, 30000.0, self.area_min, 1.0, 0.65)
        self.slider_hc = self.create_slider('hood_cutoff', 0.01, 1.0, self.hood_cutoff, 0.01, 0.55)
        self.slider_lc = self.create_slider('left_cutoff', 0.01, 1.0, self.left_cutoff, 0.01, 0.45)
        self.slider_rc = self.create_slider('right_cutoff', 0.01, 1.0, self.right_cutoff, 0.01, 0.35)
        self.slider_tc = self.create_slider('top_cutoff', 0.01, 1.0, self.right_cutoff, 0.01, 0.25)

        self.get_logger().info("Slider control node initialized.")
        self.slider_update(0)

    def create_slider(self, label, min_val, max_val, init_val, val_step, pos_y):
        slider_ax = plt.axes([0.1, pos_y, 0.8, 0.05], facecolor='lightgoldenrodyellow')
        slider = Slider(slider_ax, label, min_val, max_val, valinit=init_val, valstep=val_step)
        slider.on_changed(self.slider_update)
        return slider

    def slider_update(self, val):
        self.bin_thresh = int(self.slider_bin_thresh.val)
        self.area_max = int(self.slider_area_max.val)
        self.area_min = int(self.slider_area_min.val)
        self.hood_cutoff = self.slider_hc.val
        self.left_cutoff = self.slider_lc.val
        self.right_cutoff = self.slider_rc.val
        self.top_cutoff = self.slider_tc.val

        # self.get_logger().info(f"Updated values: bin_thresh={self.bin_thresh}, area_max={self.area_max}, area_min={self.area_min}, hood_cutoff={self.hood_cutoff}") 

        msg = Float32MultiArray()
        msg.data = list(map(float, [self.bin_thresh, self.area_max, self.area_min, self.hood_cutoff, self.left_cutoff, self.right_cutoff, self.top_cutoff]))
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
