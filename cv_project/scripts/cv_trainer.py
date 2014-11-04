#!/usr/bin/env python
import numpy as np
from sklearn.linear_model import Ridge
from geometry_msgs.msg import Twist
from image_processing import extract_data
import cv2
import rosbag
import pickle
import sys
import os


def twist_to_nparray(msg):
    return np.array([msg.linear.x, msg.angular.z])


def find_best_fit(bagfiles):
    clf = Ridge(alpha=1.0)  # TODO: auto-calibrate alpha (it's easy using a scikit-learn one-liner)
    image_data = []
    cmd_vel_data = []
    for bagfile_path in bagfiles:
        most_recent_cmd_vel = None
        bag = rosbag.Bag(bagfile_path)
        for topic, msg, t in bag.read_messages(topics=['/camera/image_raw/compressed', '/cmd_vel'], ):
            if topic == "/cmd_vel" and (most_recent_cmd_vel is not None or msg != Twist()):
                    most_recent_cmd_vel = msg
            elif topic == "/camera/image_raw/compressed" and most_recent_cmd_vel is not None:
                np_arr = np.fromstring(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
                image_data.append(extract_data(cv_image))
                cmd_vel_data.append(twist_to_nparray(most_recent_cmd_vel))

    clf.fit(image_data, cmd_vel_data)
    return clf

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Must give me the path to your bagfile directory")
    else:
        bagfile_directory = sys.argv[1]
        bagfiles = [os.path.join(bagfile_directory, file) for file in os.listdir(bagfile_directory) if file.endswith(".bag")]
        clf = find_best_fit(bagfiles)

        save_file_name = raw_input("enter filename to save processed data: ")
        with open(save_file_name, 'w') as f:
            pickle.dump(clf, f)