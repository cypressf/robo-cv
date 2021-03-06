#!/usr/bin/env python
import rospy
import signal
import sys
from sklearn.linear_model import Ridge
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
import cv_bridge
from image_processing import extract_data
import cv2
import pickle


class Controller:
    def __init__(self, saved_fit_path="saved_fit.txt"):
        '''Makes controler node and subscribes to cmd_vel node to control robot.
        Also loads trainign data
        '''
        rospy.init_node('controller', anonymous=True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = None
        self.bridge = cv_bridge.CvBridge()
        self.running = False
        self.cmd_vel = Twist()
        with open(saved_fit_path, 'r') as f:
            self.clf = pickle.load(f)
        signal.signal(signal.SIGINT, self.signal_handler)

    def image_received(self, image_message):
        """
        Process image from camera and set the desired cmd_vel
        """
        # Convert the image message to something usable by opencv
        # http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
        # Note that mono8 and bgr8 are the two image encodings expected by most OpenCV functions.
        cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        image_data = extract_data(cv_image)
        linear_velocity, angular_velocity = self.clf.predict(image_data)
        self.cmd_vel = Twist(linear=Vector3(x=linear_velocity), angular=Vector3(z=angular_velocity))
        rospy.loginfo(self.cmd_vel)

    def signal_handler(self, signal, frame):
        '''Shutdown script in case of keyboard interupt. 
        '''
        self.running = False
        self.pub.publish(Twist())
        cv2.destroyAllWindows()
        sys.exit(0)

    def run(self):
        '''Get camera images and publish the current, set point velocity
        '''
        self.running = True
        self.sub = rospy.Subscriber('camera/image_raw', Image, self.image_received)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.running:
            self.pub.publish(self.cmd_vel)
            rate.sleep()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        controller = Controller(sys.argv[1])
    else:
        controller = Controller()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass
