#!/usr/bin/env python
import cv2
import numpy as np
import rospy


def extract_data(cv_image):
    """
    Given a computer vision image, extract the data that we care about.

    :param cv_image: an opencv image to process
    :returns data: a numpy array of data we care about
    """
    rows, cols, channels = cv_image.shape
    left_half = cv_image[:, :(cols / 2), 2]
    right_half = cv_image[:, (cols / 2):, 2]
    left_sum = np.sum(left_half, 1)
    right_sum = np.sum(right_half, 1)
    diff = left_sum - right_sum
    rospy.loginfo(diff.to)
    return diff