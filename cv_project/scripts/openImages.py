import numpy as np
import cv2

image='blurrySoloCups.jpg' #image to open
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
