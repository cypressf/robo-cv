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
    
def imageProcessRedCups(imageName):
    '''Process an input image with Open CV methods to focus on red cup like shapes'''
    image=imageName
    image = cv2.imread(image,cv2.IMREAD_COLOR)
    image=cv2.medianBlur(image,9) #kernal size must be odd
    image=cv2.bilateralFilter(image,9,75,75)
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #convert to hsv image

    # define range of blue color in HSV
    lower_blue = np.array([0,50,50])
    upper_blue = np.array([50,255,255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

    #erode image to filter it:
    kernel = np.ones((6,6),np.uint8) #make 5 by 5 kernal size filter
    dilated = cv2.dilate(mask,kernel,iterations = 1)

    contours,hierarchy = cv2.findContours(dilated, 1, 2)

    cnt = contours[0]
    M = cv2.moments(cnt)
    print M

    cv2.imshow('img',image)
    #cv2.imshow('dilated image', dilated)

    cv2.imshow('mask',mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    
if __name__=='__main__':
    imageProcessRedCups('blurrySoloCups.jpg')
