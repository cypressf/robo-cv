#!/usr/bin/env python
import cv2
import numpy as np


def closeImages():
    """
    Close images opened by Open CV, addresses the different method of closing
    images based on command line or called usage.

    INPUT: none
    OUTPUT: none
    **Closes all open cv2 images either on a keystroke (comman line use) or immediately (called)
    """
    if __name__ == '__main__':
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        cv2.waitKey(3)


def returnImg(return_img):
    """
    Returns the image as a numpy matrix after closing all open images

    INPUT: image to be returned
    OUTPUT: singular value matrix representing the image
    """
    closeImages()
    imageArray = np.asarray(return_img)
    imageMatrix = np.matrix(imageArray)
    return np.linalg.svd(imageMatrix, compute_uv=0)  #setting compute_uv to zero only computes the singular value
    #TODO: check if singular matrix is what we need/want


def extract_data(cv_image):
    """
    Given a computer vision image, extract the data that we care about.

    :param cv_image: an opencv image to process
    :returns data: a numpy array of data we care about
    """
    return imageProcessRedCups(cv_image)


def hsv_test_blue(cv_image):
    # define range of blue color in HSV
    lower_blue = np.array([90, 10, 0])
    upper_blue = np.array([160, 255, 255])
    return hsv_test(cv_image, lower_blue, upper_blue)


def hsv_test_red(cv_image):
    # define range of red color in HSV
    lower_red = np.array([150, 30, 30])
    upper_red = np.array([200, 255, 255])
    return hsv_test(cv_image, lower_red, upper_red)


def normalize(np_array):
    norm = np.linalg.norm(np_array)
    if norm == 0:
        return norm
    else:
        return np_array / norm


def mouse_event(event, x, y, flag, im):
    """
    Print the pixel values  of the image where it's clicked on.
    """
    global colorImgInput
    if event == cv2.EVENT_FLAG_LBUTTON:
        print
        colorImgInput[y, x, :]


def hsv_test(cv_image, lower_color, upper_color):
    """
    Mask the image so only certain colors show up (within a range of colors), then add the
    intensities of the pixels in all columns to get a vector of values that represents
    the intensities of that color across the horizontal axis of the image.
    """
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only a certain range of colors
    mask = cv2.inRange(hsv_image, lower_color, upper_color)

    # Bitwise-AND mask and original image
    mask_results = cv2.bitwise_and(cv_image, cv_image, mask=mask)
    h, s, v = cv2.split(mask_results)

    cv2.imshow("image", mask_results)
    closeImages()
    return normalize(np.array(np.sum(v, axis=0)))


def colorImgPreProcess(image):
    """
    Prepare images to be analyzed in binary form by appling generic filtering.
    This makes them easier to work with and prettier.

    INPUT: image for pre-processing. Should be in color, though b&w ahould work.
    OUTPUT: returns a RGB image which has been filtered and looks nicer.
    """
    #do processing on the image while it's still in color
    image = cv2.medianBlur(image, 7)  #kernal size must be odd
    image = cv2.bilateralFilter(image, 9, 75, 75)

    #    cv2.imshow('img',image)
    #    closeImages()
    return image


def imageProcessBlueCups(image):
    """
    Process an input image with Open CV methods to focus on blue cup like shapes

    INPUT: image
    OUTPUT: binary image better suited for prcessing with the ridge regression
    """

    #pre-process image while it's in color
    colorImgInput = colorImgPreProcess(image)

    # define range of blue color for HSV
    lower_blue = np.array([170, 30, 50])
    upper_blue = np.array([270, 360, 255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(colorImgInput, lower_blue, upper_blue)

    cv2.imshow('blue cup mask', mask)
    closeImages()


def imageProcessRedCups(image):
    """
    Process an input image with Open CV methods to focus on red cup like shapes

    INPUT: image
    OUTPUT: matrix of singular values of the binary image for prcessing
        with the ridge regression
    """
    #    global colorImgInput #from mouse clicking stuff
    #pre-process image while it's in color
    image = colorImgPreProcess(colorImgPreProcess(image))  #run color filtering 2x

    # define range of red color for HSV
    lower_redColor = np.array([30, 30, 160])
    upper_redColor = np.array([70, 80, 230])

    # Threshold the RGB image to get only red colors
    RGBMask = cv2.inRange(image, lower_redColor, upper_redColor)

    #dialate image to filter it:
    kernel = np.ones((6, 6), np.uint8)  #make 5 by 5 kernal size filter
    rgbDilated = cv2.dilate(RGBMask, kernel, iterations=1)

    #    cv2.setMouseCallback("color mask",mouse_event,[])
    cv2.imshow("color mask", RGBMask)
    cv2.imshow('dilated image', rgbDilated)

    return returnImg(rgbDilated)


if __name__ == '__main__':
    image = cv2.imread('cups.jpg', cv2.IMREAD_COLOR)
    #    print image.shape
    closeImages()
    imageProcessRedCups(image)
#    imageProcessBlueCups(image)
