#!/usr/bin/env python
import rospy
import signal
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv_bridge


class Controller:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = None
        self.running = False
        signal.signal(signal.SIGINT, self.signal_handler)

    def image_received(self, img):
        rospy.loginfo("Image received! %s" % str(img))

    def move(self):
        return Twist()

    def signal_handler(self, signal, frame):
        self.running = False
        self.pub.publish(Twist())
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