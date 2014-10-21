#!/usr/bin/env python
import cv2
import numpy as np


def extract_data(cv_image):
    """
    Given a computer vision image, extract the data that we care about.

    :param cv_image: an opencv image to process
    :returns data: a numpy array of data we care about
    """
    rows, cols, channels = cv_image.shape
    # TODO
    return np.array([1,2,3])