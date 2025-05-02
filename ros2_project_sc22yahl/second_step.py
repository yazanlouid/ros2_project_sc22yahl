# Exercise 2 - detecting two colours, and filtering out the third colour and background.

import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal


class colourDetector(Node):
    def __init__(self):
        super().__init__('colour_detector')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg):

        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return
        
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define color ranges in HSV
        # Red
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # Green
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])

        # Blue
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])

        # Masks
        red_mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Draw contours
        for mask, color_name, color in [(red_mask, "Red", (0, 0, 255)),
                                        (green_mask, "Green", (0, 255, 0)),
                                        (blue_mask, "Blue", (255, 0, 0))]:
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:  # filter small noise
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), color, 2)
                    cv2.putText(cv_image, color_name, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        cv2.imshow("Detected Colours", cv_image)
        cv2.waitKey(1)

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():

    def signal_handler(sig, frame):
        rclpy.shutdown()
        cv2.destroyAllWindows()

    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init()
    node = colourDetector()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            pass
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
    
# Check if the node is executing in the main path
if __name__ == '__main__':
    main()
