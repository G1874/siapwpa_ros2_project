# Można spróbować jechać w drugą stronę, żeby nie było od razu widać wykrywania okien
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

import cv2
from cv_bridge import CvBridge

import numpy as np
import tensorflow as tf


class SignDetectorAndClassifier(Node):
    def __init__(self):
        super().__init__("sign_detector_classifier")
        self.subscription = self.create_subscription(
            Image, "/front_camera_sensor/image", self.image_callback, 10
        )
        self.bridge = CvBridge()

        self.r_r = 100
        self.r_g = 55
        self.r_b = 55

        self.b_r = 51
        self.b_g = 93
        self.b_b = 55

        self.slider_sub = self.create_subscription(
            Float32MultiArray, "/color_slider_values", self.slider_callback, 10
        )

        self.model = tf.saved_model.load(
            "/home/developer/ros2_ws/src/autonomous_vehicle/autonomous_vehicle/good_model"
        )
        self.publisher = self.create_publisher(Image, "/sign_trafficking", 10)
        self.class_labels = {
            1: "Speed limit (20km/h)",
            2: "Speed limit (30km/h)",
            3: "Speed limit (50km/h)",
            4: "Speed limit (60km/h)",
            5: "Speed limit (70km/h)",
            6: "Speed limit (80km/h)",
            7: "End of speed limit (80km/h)",
            8: "Speed limit (100km/h)",
            9: "Speed limit (120km/h)",
            10: "No passing",
            11: "No passing vehicles over 3.5 tons",
            12: "Right-of-way at intersection",
            13: "Priority road",
            14: "Yield",
            15: "Stop",
            16: "No vehicles",
            17: "Vehicles > 3.5 tons prohibited",
            18: "No entry",
            19: "General caution",
            20: "Dangerous curve left",
            21: "Dangerous curve right",
            22: "Double curve",
            23: "Bumpy road",
            24: "Slippery road",
            25: "Road narrows on the right",
            26: "Road work",
            27: "Traffic signals",
            28: "Pedestrians",
            29: "Children crossing",
            30: "Bicycles crossing",
            31: "Beware of ice/snow",
            32: "Wild animals crossing",
            33: "End speed + passing limits",
            34: "Turn right ahead",
            35: "Turn left ahead",
            36: "Ahead only",
            37: "Go straight or right",
            38: "Go straight or left",
            39: "Keep right",
            40: "Keep left",
            41: "Roundabout mandatory",
            42: "End of no passing",
            43: "End no passing for vehicles over 3.5 tons",
        }

    def slider_callback(self, msg):
    #     self.r = int(msg.data[0])
    #     self.g = int(msg.data[1])
    #     self.b = int(msg.data[2])
        pass

    def preprocess_image(self, frame):
        """
        Preprocess the frame for the classifier:
        Resize, normalize, and add batch dimensions.
        """
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.resize(frame, (30, 30))
        frame = np.array(frame) / 255.0
        frame = np.expand_dims(frame, axis=0)
        return tf.convert_to_tensor(frame, dtype=tf.float32)

    def classify_sign(self, cropped_region):
        input_tensor = self.preprocess_image(cropped_region)
        output = self.model.signatures["serving_default"](input_tensor)
        predictions = output["output_0"].numpy()
        pred = np.argmax(predictions, axis=1)[0]
        confidence = np.max(predictions)
        sign = self.class_labels[pred + 1]
        return sign, confidence

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if frame is None:
            self.get_logger().info("No frame received")
            return

        red = frame[:, :, 2]
        green = frame[:, :, 1]
        blue = frame[:, :, 0]

        red_signs = np.logical_and(
            np.logical_and(red > self.r_r, green < self.r_g), blue < self.r_b
        )
        blue_signs = np.logical_and(
            np.logical_and(red < self.b_r, green < self.b_g), blue > self.b_b
        )
        signs = np.logical_or(red_signs, blue_signs)
        _, _, stats, _ = cv2.connectedComponentsWithStats(signs.astype(np.uint8) * 255)

        # ignore the first stat which is the background
        for x, y, w, h, a in stats[1:]:
            # ignore small regions and regions that are not basically square
            if a >= 100 and abs(w - h) < 5:
                cropped_region = frame[y : y + h, x : x + w]
                label, confidence = self.classify_sign(cropped_region)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    f"{label} ({confidence:.2f})",
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 0, 0),
                    2,
                )
        ##########################
        cv2.imshow("signs", frame)
        cv2.waitKey(1)
        ##########################
        frame = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher.publish(frame)



def main(args=None):
    rclpy.init(args=args)
    node = SignDetectorAndClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
