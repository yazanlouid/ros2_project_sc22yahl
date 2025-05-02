# Exercise 1 - Display an image of the camera feed to the screen

#from __future__ import division
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
        super().__init__('colour_identifier')
        
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', # defualt camera topic
            self.callback,
            10
        )
        self.subscription # prevent unused variable warning
        
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {str(e)}")
        

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():

    def signal_handler(sig, frame):
        rclpy.shutdown()
    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
    node = colourIdentifier()


    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            pass
    except ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
    

# Check if the node is executing in the main path
if __name__ == '__main__':
    main()
