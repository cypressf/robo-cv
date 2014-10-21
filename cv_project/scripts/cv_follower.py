#!/usr/bin/env python
import rospy
import signal
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv_bridge
import cv2


class Controller:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = None
        self.bridge = cv_bridge.CvBridge()
        self.running = False
        signal.signal(signal.SIGINT, self.signal_handler)

    def image_received(self, image_message):
        """
        Process image and set the desired cmd_vel
        """
        # Convert the image message to something usable by opencv
        # http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
        # Note that mono8 and bgr8 are the two image encodings expected by most OpenCV functions.
        cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        rows, cols, channels = cv_image.shape
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

    def move(self):
        return Twist()

    def signal_handler(self, signal, frame):
        self.running = False
        self.pub.publish(Twist())
        cv2.destroyAllWindows()
        sys.exit(0)

    def run(self):
        self.running = True
        self.sub = rospy.Subscriber('camera/image_raw', Image, self.image_received)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.running:
            movement = self.move()
            self.pub.publish(movement)
            rate.sleep()

if __name__ == "__main__":
    try:
        controller = Controller()
        controller.run()
    except rospy.ROSInterruptException:
        pass