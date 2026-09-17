import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class ColorDetector(Node):

    def __init__(self):
        super().__init__('color_detector')

        # Gazebo camera
        self.image_sub = self.create_subscription(
            Image,
            '/sorting_camera/image',
            self.image_callback,
            qos_profile_sensor_data
        )

        # Publish detected colour
        self.color_pub = self.create_publisher(
            String,
            '/detected_color',
            10
        )

        self.bridge = CvBridge()

        # Detection state
        self.candidate = 'NONE'
        self.candidate_count = 0
        self.required_frames = 5

        self.current_color = 'NONE'
        self.previous_color = 'NONE'

        # Publish current result twice per second.
        # This means ros2 topic echo will always see messages.
        self.publish_timer = self.create_timer(
            0.5,
            self.publish_current_color
        )

        self.get_logger().info(
            'Color Detector started.'
        )
        self.get_logger().info(
            'Camera: /sorting_camera/image'
        )
        self.get_logger().info(
            'Publisher: /detected_color'
        )

    def image_callback(self, msg):

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as ex:
            self.get_logger().error(
                f'CV Bridge error: {ex}'
            )
            return

        height, width, _ = frame.shape

        # Central inspection region
        x1 = int(width * 0.25)
        x2 = int(width * 0.75)

        y1 = int(height * 0.25)
        y2 = int(height * 0.75)

        roi = frame[
            y1:y2,
            x1:x2
        ]

        hsv = cv2.cvtColor(
            roi,
            cv2.COLOR_BGR2HSV
        )

        # ---------------------------
        # BLUE
        # ---------------------------

        blue_mask = cv2.inRange(
            hsv,
            np.array([100, 100, 70]),
            np.array([140, 255, 255])
        )

        # ---------------------------
        # GREEN
        # ---------------------------

        green_mask = cv2.inRange(
            hsv,
            np.array([40, 70, 70]),
            np.array([85, 255, 255])
        )

        # ---------------------------
        # RED
        # ---------------------------

        red_mask_1 = cv2.inRange(
            hsv,
            np.array([0, 100, 70]),
            np.array([10, 255, 255])
        )

        red_mask_2 = cv2.inRange(
            hsv,
            np.array([170, 100, 70]),
            np.array([180, 255, 255])
        )

        red_mask = cv2.bitwise_or(
            red_mask_1,
            red_mask_2
        )

        blue_pixels = cv2.countNonZero(
            blue_mask
        )

        green_pixels = cv2.countNonZero(
            green_mask
        )

        red_pixels = cv2.countNonZero(
            red_mask
        )

        minimum_pixels = 100

        detected = 'NONE'

        largest = max(
            blue_pixels,
            green_pixels,
            red_pixels
        )

        if largest >= minimum_pixels:

            if largest == blue_pixels:
                detected = 'BLUE'

            elif largest == green_pixels:
                detected = 'GREEN'

            elif largest == red_pixels:
                detected = 'RED'

        # Require same result for 5 consecutive frames
        if detected == self.candidate:
            self.candidate_count += 1

        else:
            self.candidate = detected
            self.candidate_count = 1

        if self.candidate_count >= self.required_frames:

            self.current_color = detected

            if self.current_color != self.previous_color:

                if self.current_color == 'BLUE':
                    self.get_logger().warn(
                        'BLUE DEFECT DETECTED!'
                    )

                elif self.current_color in [
                    'RED',
                    'GREEN'
                ]:
                    self.get_logger().info(
                        f'{self.current_color} product detected.'
                    )

                else:
                    self.get_logger().info(
                        'Inspection zone clear.'
                    )

                self.previous_color = (
                    self.current_color
                )

    def publish_current_color(self):

        msg = String()
        msg.data = self.current_color

        self.color_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = ColorDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
