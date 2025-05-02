# Exercise 4 - following a colour (green) and stopping upon sight of another (blue).

#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.exceptions import ROSInterruptException
import signal

class Robot(Node):
    def __init__(self):
        super().__init__('robot')

        # Publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Colour detection settings
        self.sensitivity = 10
        self.colour1_flag = 0  # Green
        self.colour2_flag = 0  # Blue

        self.moveForwardFlag = False
        self.moveBackwardFlag = False

        # Thresholds
        self.min_area = 1000  # Area to start responding
        self.too_close_area = 5000  # Area that means "too close"

        # Movement commands
        self.move_forward_cmd = Twist()
        self.move_forward_cmd.linear.x = 0.2

        self.move_backward_cmd = Twist()
        self.move_backward_cmd.linear.x = -0.2

        self.stop_cmd = Twist()  # All zeros

        # Camera subscriber
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Green
        lower_green = np.array([60 - self.sensitivity, 100, 100])
        upper_green = np.array([60 + self.sensitivity, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        green_result = cv2.bitwise_and(image, image, mask=green_mask)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Blue
        lower_blue = np.array([110 - self.sensitivity, 100, 100])
        upper_blue = np.array([110 + self.sensitivity, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Reset flags
        self.colour1_flag = 0
        self.colour2_flag = 0
        self.moveForwardFlag = False
        self.moveBackwardFlag = False

        # Process green contours
        if green_contours:
            c = max(green_contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > self.min_area:
                self.colour1_flag = 1
                if area > self.too_close_area:
                    self.moveBackwardFlag = True
                else:
                    self.moveForwardFlag = True

        # Process blue contours
        if blue_contours:
            c = max(blue_contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > self.min_area:
                self.colour2_flag = 1  # STOP if blue is seen

        # Display
        cv2.imshow('camera_Feed', image)
        cv2.waitKey(3)

    def walk_forward(self):
        self.publisher.publish(self.move_forward_cmd)

    def walk_backward(self):
        self.publisher.publish(self.move_backward_cmd)

    def stop(self):
        self.publisher.publish(self.stop_cmd)

def main():
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    rclpy.init(args=None)
    global robot
    robot = Robot()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            if robot.colour2_flag == 1:
                robot.stop()
            elif robot.colour1_flag == 1:
                if robot.moveBackwardFlag:
                    robot.walk_backward()
                elif robot.moveForwardFlag:
                    robot.walk_forward()
            else:
                robot.stop()

            time.sleep(0.1)

    except ROSInterruptException:
        pass

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

