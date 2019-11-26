import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tracker

rospy.init_node("image_processing")
rate = rospy.Rate(30)

#tracker = cv2.Tracker_create('MIL')
bridge = CvBridge()

## We will hardcode this for now
## and implement YOLO detection algorithm
bbox = (287, 23, 86, 320)
#bbox = cv2.selectROI(frame, False)
p1 = (int(bbox[0]), int(bbox[1]))
p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))

def image_cb(ros_image):
    global i
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, 'bgr8')
    except CvBridgeError as e:
        print(e)
    
    tracker.track_and_display( cv_image, display = True)
    #cv2.rectangle(cv_image, p1, p2, (255,0,0), 2, 1)
    #cv2.imshow("Image Window", cv_image)
    #cv2.waitKey(3)

image_sub = rospy.Subscriber("/iris_0/camera_red_iris/image_raw", Image, image_cb)

while(not rospy.is_shutdown()):
    rate.sleep()
