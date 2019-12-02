import cv2
import numpy as np
# Read image. 
img = cv2.imread('./autonomy/temp/test2.png', cv2.IMREAD_COLOR)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# Range for lower red
lower_red = np.array([0,120,70])
upper_red = np.array([10,255,255])
mask1 = cv2.inRange(hsv, lower_red, upper_red)

# Range for upper range
lower_red = np.array([170,120,70])
upper_red = np.array([180,255,255])
mask2 = cv2.inRange(hsv,lower_red,upper_red)
# Generating the final mask to detect red color
mask1 = mask1+mask2
mask1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))
mask1 = cv2.morphologyEx(mask1, cv2.MORPH_DILATE, np.ones((3,3),np.uint8))

#creating an inverted mask to segment out the cloth from the frame
#mask2 = cv2.bitwise_not(mask1)
#Segmenting the cloth out of the frame using bitwise and with the inverted mask
res1 = cv2.bitwise_and(img,img,mask=mask1)
#cv2.imshow("Detected Circle", res1)
# Convert to grayscale. 
gray = cv2.cvtColor(res1, cv2.COLOR_BGR2GRAY) 
# Blur using 3 * 3 kernel. 
gray_blurred = cv2.blur(gray, (3, 3))
# Apply Hough transform on the blurred image. 
detected_circles = cv2.HoughCircles(gray_blurred,  
                   cv2.HOUGH_GRADIENT, 1, 10, param1 = 50, 
               param2 = 10, minRadius = 1, maxRadius = 40)

# Draw circles that are detected. 
if detected_circles is not None:
    # Convert the circle parameters a, b and r to integers. 
    detected_circles = np.uint16(np.around(detected_circles))
    for pt in detected_circles[0, :]:
        a, b, r = pt[0], pt[1], pt[2] 
        # Draw the circumference of the circle. 
        cv2.circle(res1, (a, b), r, (0, 255, 0), 2)
        # Draw a small circle (of radius 1) to show the center. 
        cv2.circle(res1, (a, b), 1, (0, 0, 255), 3) 
cv2.imshow("Detected Circle", res1)
cv2.waitKey(0)