#!/usr/bin/env python
import numpy as np
from sklearn.linear_model import Ridge
from geometry_msgs.msg import Twist
from image_processing import extract_data
import cv2
import rosbag


def twist_to_nparray(msg):
    return np.array([msg.linear.x, msg.angular.z])


if __name__ == "__main__":
    most_recent_cmd_vel = Twist()
    bag = rosbag.Bag('/home/cypressf/robo-cv/rosbag/2014-10-21-16-07-47.bag')
    clf = Ridge(alpha=1.0)  # TODO: auto-calibrate alpha (it's easy using a scikit-learn one-liner)
    image_data = []
    cmd_vel_data = []
    for topic, msg, t in bag.read_messages(topics=['/camera/image_raw/compressed', '/cmd_vel'], ):
        if topic == "/cmd_vel":
            most_recent_cmd_vel = msg
        elif topic == "/camera/image_raw/compressed":
            np_arr = np.fromstring(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
            image_data.append(extract_data(cv_image))
            cmd_vel_data.append(twist_to_nparray(most_recent_cmd_vel))

    image_data_array = np.array(image_data)
    cmd_vel_array = np.array(cmd_vel_data)

    clf.fit(image_data,cmd_vel_data)