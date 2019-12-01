import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tracker

rospy.init_node("image_processing")
rate = rospy.Rate(30)

bridge = CvBridge()

def segment( img):
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

    cv2.imshow("Red segmentation" ,res1)
    cv2.imwrite("test.png", res1)
    cv2.waitKey(3)

def image_cb(ros_image):
    global i
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, 'bgr8')
    except CvBridgeError as e:
        print(e)
    
    # Uncomment this to enable tracking
    #tracker.track_and_display( cv_image, display = True)
    
    # Image segmentation
    segment(cv_image)

image_sub = rospy.Subscriber("/iris_0/camera_red_iris/image_raw", Image, image_cb)

while(not rospy.is_shutdown()):
    rate.sleep()
