#!/usr/bin/env python
import cv2
import numpy as np

def closeImages():
    '''Close images opened by Open CV, addresses the different method of closing
    images based on command line or called usage.
    
    INPUT: none
    OUTPUT: none
    **Closes all open cv2 images either on a keystroke (comman line use) or immediately (called)'''
    if __name__=='__main__':
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        cv2.waitKey(3)

def extract_data(cv_image):
    """
    Given a computer vision image, extract the data that we care about.

    :param cv_image: an opencv image to process
    :returns data: a numpy array of data we care about
    """
    return imageProcessRedCups(cv_image)


def mouse_event(event,x,y,flag,im):
    global colorImgInput
    if event == cv2.EVENT_FLAG_LBUTTON:
        print colorImgInput[y,x,:]

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

def colorImgPreProcess(image):
    #do processing on the image while it's still in color
#    image=cv2.medianBlur(image,7) #kernal size must be odd
#    image=cv2.bilateralFilter(image,9,75,75)
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #convert to hsv image

    cv2.imshow('img',image)
    closeImages()
    return hsv_image
    
def imageProcessBlueCups(image):
    '''Process an input image with Open CV methods to focus on blue cup like shapes
    
    INPUT: image
    OUTPUT: binary image better suited for prcessing with the ridge regression'''
    
    #pre-process image while it's in color
    colorImgInput=colorImgPreProcess(image) 
    
    # define range of blue color for HSV
    lower_blue = np.array([170,30,50])
    upper_blue = np.array([270,360,255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(colorImgInput, lower_blue, upper_blue)
    
    cv2.imshow('blue cup mask',mask)
    closeImages()
    

def imageProcessRedCups(image):
    '''Process an input image with Open CV methods to focus on red cup like shapes
    
    INPUT: image
    OUTPUT: binary image better suited for prcessing with the ridge regression'''
    
    #pre-process image while it's in color
    global colorImgInput
    colorImgInput=colorImgPreProcess(image) 
    
    # define range of red color for HSV
    lower_red1 = np.array([0,50,50])
    upper_red1 = np.array([40,105,85])
    lower_red2 = np.array([220,0,0])
    upper_red2 = np.array([255,255,255])

    # Threshold the HSV image to get only red colors
    mask1 = cv2.inRange(colorImgInput, lower_red1, upper_red1)
    #mask2 = cv2.inRange(colorImgInput, lower_red2, upper_red2)
   # mask=cv2.add(mask1,mask2)
    mask=mask1

    #dialate image to filter it:
    kernel = np.ones((6,6),np.uint8) #make 5 by 5 kernal size filter
    dilated = cv2.dilate(mask1,kernel,iterations = 1)

    #Contor stuff, in dev
    contours,hierarchy = cv2.findContours(dilated, 1, 2)
    cnt = contours[0]
    M = cv2.moments(cnt)
    print M
    
    cv2.drawContours(mask, contours, -1, (40,255,0), 3)

    cv2.imshow('mask 1',mask1)
    cv2.setMouseCallback("mask 1",mouse_event,[])
   # cv2.imshow('mask 2',mask2)
   # cv2.imshow('mask',mask)
#    cv2.imshow('drawn contors', contors)
    #cv2.imshow('dilated image', dilated)

    
    
    closeImages()

    
    
if __name__=='__main__':
    image = cv2.imread('cups.jpg',cv2.IMREAD_COLOR)
    print image.shape
    closeImages()
    imageProcessRedCups(image)
#    imageProcessBlueCups(image)
