# Exercise 3 - If green object is detected, and above a certain size, then send a message (print or use lab2)

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



class colourIdentifier(Node):
    def __init__(self):
        super().__init__('cI')
        
        self.green_detected = False
        self.sensitivity = 10

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning
        
    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        # Show original image
        cv2.namedWindow('camera_Feed', cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed', image)
        cv2.resizeWindow('camera_Feed', 320, 240)
        cv2.waitKey(3)

        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])

        mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)

        result = cv2.bitwise_and(image, image, mask=mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > 500:  # Adjustable threshold
                M = cv2.moments(c)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    (x, y), radius = cv2.minEnclosingCircle(c)
                    center = (int(x), int(y))
                    cv2.circle(image, center, int(radius), (0, 255, 0), 2)

                    self.green_detected = True
                    print("Green object detected!")

        self.green_detected = False

        cv2.imshow('Green Detection', result)
        cv2.waitKey(3)

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():
    def signal_handler(sig, frame):
        rclpy.shutdown()

    
    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
   
    cI = colourIdentifier()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(cI,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            continue
    except ROSInterruptException:
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()


# Check if the node is executing in the main path
if __name__ == '__main__':
    main()
