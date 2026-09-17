import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class ColorDetector(Node):
    """
    ROS 2 Node for real-time computer vision processing.
    Subscribes to a camera topic, detects colored objects (Red, Green, Blue) 
    passing on a conveyor belt, and publishes the detected product type.
    """

    def __init__(self):
        super().__init__('color_detector')

        # Subscribe to the Gazebo camera image topic.
        # qos_profile_sensor_data is specifically designed for high-throughput, 
        # lossy data like camera feeds to prevent network congestion.
        self.image_sub = self.create_subscription(
            Image,
            '/sorting_camera/image',
            self.image_callback,
            qos_profile_sensor_data
        )

        # Publisher to announce the currently detected color to the rest of the system
        # (e.g., to trigger MoveIt trajectory planning).
        self.color_pub = self.create_publisher(
            String,
            '/detected_color',
            10
        )

        # Utility to convert between ROS Image messages and OpenCV image formats
        self.bridge = CvBridge()

        # --- State Machine & Debouncing Variables ---
        # To prevent false positives from camera noise or partial objects entering the frame,
        # we require the same color to be detected across multiple consecutive frames.
        self.candidate = 'NONE'
        self.candidate_count = 0
        self.required_frames = 5  # Must see the same color for 5 frames to confirm detection

        self.current_color = 'NONE'
        self.previous_color = 'NONE'

        # Timer to continuously broadcast the current system state at 2 Hz.
        # This ensures that even if no new object arrives, new subscribers know the current state.
        self.publish_timer = self.create_timer(
            0.5,
            self.publish_current_color
        )

        self.get_logger().info('Color Detector initialized and running.')
        self.get_logger().info('Subscribed to Camera: /sorting_camera/image')
        self.get_logger().info('Publishing to Topic: /detected_color')

    def image_callback(self, msg):
        """
        Callback function executed every time a new image frame is received from Gazebo.
        """
        try:
            # Convert ROS Image message to an OpenCV BGR numpy array
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as ex:
            self.get_logger().error(f'CV Bridge error: {ex}')
            return

        height, width, _ = frame.shape

        # --- Define Region of Interest (ROI) ---
        # We only inspect the center 50% of the image to ignore the edges of the conveyor
        # and prevent detecting objects that are just entering/leaving the camera view.
        x1 = int(width * 0.25)
        x2 = int(width * 0.75)
        y1 = int(height * 0.25)
        y2 = int(height * 0.75)

        # Crop the frame to the defined ROI
        roi = frame[y1:y2, x1:x2]

        # Convert the cropped image from BGR to HSV color space.
        # HSV is much more robust against lighting/shadow changes than BGR/RGB.
        hsv = cv2.cvtColor(
            roi,
            cv2.COLOR_BGR2HSV
        )

        # ---------------------------
        # Color Thresholding (Masks)
        # ---------------------------

        # 1. BLUE (Our target defect object)
        blue_mask = cv2.inRange(
            hsv,
            np.array([100, 100, 70]),
            np.array([140, 255, 255])
        )

        # 2. GREEN
        green_mask = cv2.inRange(
            hsv,
            np.array([40, 70, 70]),
            np.array([85, 255, 255])
        )

        # 3. RED
        # Note: In OpenCV, Hue values wrap around at 180. Therefore, true red 
        # spans across 0-10 and 170-180. We need two masks combined together.
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
        red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

        # Count how many pixels match each color in the ROI
        blue_pixels = cv2.countNonZero(blue_mask)
        green_pixels = cv2.countNonZero(green_mask)
        red_pixels = cv2.countNonZero(red_mask)

        # Minimum pixel threshold to filter out tiny noise specs
        minimum_pixels = 100
        detected = 'NONE'

        # Find the color with the highest pixel count in the current frame
        largest = max(blue_pixels, green_pixels, red_pixels)

        # Classify the object if the largest pixel count exceeds our noise threshold
        if largest >= minimum_pixels:
            if largest == blue_pixels:
                detected = 'BLUE'
            elif largest == green_pixels:
                detected = 'GREEN'
            elif largest == red_pixels:
                detected = 'RED'

        # --- Temporal Filter (Debouncing Logic) ---
        # Ensure the detection is stable over 'self.required_frames' consecutive frames
        if detected == self.candidate:
            self.candidate_count += 1
        else:
            # If the color changes, reset the counter and track the new candidate
            self.candidate = detected
            self.candidate_count = 1

        # Once the candidate has been seen consistently enough, update the actual state
        if self.candidate_count >= self.required_frames:
            
            self.current_color = detected

            # Only log terminal output if the state has genuinely changed from the previous state
            if self.current_color != self.previous_color:

                if self.current_color == 'BLUE':
                    self.get_logger().warn('BLUE DEFECT DETECTED! Triggering sorting sequence...')
                elif self.current_color in ['RED', 'GREEN']:
                    self.get_logger().info(f'Standard {self.current_color} product passed.')
                else:
                    self.get_logger().info('Inspection zone clear. Waiting for next product.')

                self.previous_color = self.current_color

    def publish_current_color(self):
        """
        Timer callback to publish the confirmed color to the ROS network.
        """
        msg = String()
        msg.data = self.current_color
        self.color_pub.publish(msg)


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Instantiate the node
    node = ColorDetector()

    try:
        # Keep the node alive and listening to callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Allow graceful shutdown via Ctrl+C
        pass

    # Cleanup procedures
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
