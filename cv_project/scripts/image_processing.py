#!/usr/bin/env python
import cv2
import numpy as np


def extract_data(cv_image):
    """
    Given a computer vision image, extract the data that we care about.

    :param cv_image: an opencv image to process
    :returns data: a numpy array of data we care about
    """
    return hsv_test(cv_image)


def hsv_test(cv_image):
    """
    Mask the image so only blue things show up (within a range of blues), then add the
    intensities of the pixels in all columns to get a vector of values that represents
    the blueness across the horizontal axis of the image.
    """
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_blue = np.array([90, 10, 0])
    upper_blue = np.array([160, 255, 255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

    # Bitwise-AND mask and original image
    mask_results = cv2.bitwise_and(cv_image, cv_image, mask=mask)
    h, s, v = cv2.split(mask_results)

    cv2.imshow("image", mask_results)
    cv2.waitKey(3)
    return np.array(np.sum(v, axis=0))