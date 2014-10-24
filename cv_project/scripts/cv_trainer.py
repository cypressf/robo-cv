#!/usr/bin/env python
import numpy as np
from sklearn.linear_model import Ridge
from geometry_msgs.msg import Twist
from image_processing import extract_data
import cv2
import rosbag


def twist_to_nparray(msg):
    return np.array([msg.linear.x, msg.linear.y, msg.angular.x, msg.angular.y])


if __name__ == "__main__":
    most_recent_cmd_vel = Twist()
    bag = rosbag.Bag('/home/cypressf/robo-cv/rosbag/2014-10-21-16-07-47.bag')
    clf = Ridge(alpha=1.0)
    # X = np.array([])
    for topic, msg, t in bag.read_messages(topics=['/camera/image_raw/compressed', '/cmd_vel'], ):
        if topic == "/cmd_vel":
            most_recent_cmd_vel = msg
        elif topic == "/camera/image_raw/compressed":
            np_arr = np.fromstring(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)
            # TODO: somehow append this x and y data to a list of Xs and Ys that we can
            # TODO: feed into clf.fit(X, Y)
            x = extract_data(cv_image)
            y = twist_to_nparray(most_recent_cmd_vel)
    # clf.fit(X, Y)