import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
# import tracker

rospy.init_node("ip")
rate = rospy.Rate(30)
bridge = CvBridge()

K = np.array([[476.7030836014194, 0.0, 400.5],
         [0.0, 476.7030836014194, 400.5],
         [0.0, 0.0, 1.0]])

R_cu = np.array([
    [0,0,1,0],
    [0,1,0,0],
    [-1,0,0,-0.07],
    [0,0,0,1]])
inv_R_cu = np.linalg.inv(R_cu)
f = K[0,0]
R = 70
c_x = K[0,2]
c_y = K[1,2]

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
    #cv2.imshow("Detected Circle", res1)

    ################## WE CAN DO THRESHOLDING ##############

    ########################################################
    print("Detecting circles.")
    gray_blurred = cv2.cvtColor(res1, cv2.COLOR_BGR2GRAY)
    # Apply Hough transform on the blurred image. 
    detected_circles = cv2.HoughCircles(gray_blurred,  
                    cv2.HOUGH_GRADIENT, 1, 10, param1 = 50, 
                param2 = 10, minRadius = 1, maxRadius = 40)
    
    if detected_circles is not None:
        # Convert the circle parameters a, b and r to integers. 
        print("Found")
        detected_circles = np.uint16(np.around(detected_circles)) 
        
        for pt in detected_circles[0, :]: 
            a, b, r = pt[0], pt[1], pt[2] 
            
            #print(a,b,r)
            img_to_uav(a,b,r)
            # Draw the circumference of the circle. 
            cv2.circle(res1, (a, b), r, (0, 255, 0), 2)
            # Draw a small circle (of radius 1) to show the center. 
            cv2.circle(res1, (a, b), 1, (0, 0, 255), 3)

def img_to_uav( x_i, y_i, r):

    # pixel to mm
    print(x_i)
    scale = 0.493
    x_i = x_i * scale
    y_i = y_i * scale
    r = r  * scale
    #r = 11 * scale
    # Camera coordinate system
    x_cc = np.array([
        f * R / r,
        1/f * (y_i - (f * R * c_y / r)),
        1/f * (x_i - (f * R * c_x / r)),
        1
    ])

    print( x_cc)

    # x in ua0v coordinate
    x_uc = np.dot(R_cu , x_cc)

    # publish 
    pt = Point()
    pt.x = x_uc[0]/1000
    pt.y = x_uc[1]/1000
    pt.z = x_uc[2]/1000
    rogue_pub.publish(pt)
    

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
rogue_pub = rospy.Publisher("/rogue",Point,queue_size=3)
while(not rospy.is_shutdown()):
    rate.sleep()
