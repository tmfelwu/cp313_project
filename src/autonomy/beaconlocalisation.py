#!/usr/bin/env python


'''
    This node subscribes to gazebo's position messages for the uav and the beacon.
    It additionally computes R,r,theta and phi . 
    R - the radial distance between the beacon and uav  in vertical plane ,
    r - the radial distance between the beacon and uav  in horizontal plane ,
    theta - the angle made by  line joining uav and beacon in horizontal plane, 
    phi - the angle made by  line joining uav and beacon in vertical  plane .
'''



import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from cp313_project.msg import ProjectMsg
import math
import numpy as np



''' you may tweak these parameters '''
''' Beacon detection thresholds in meters'''
beacon_threshold = 300
radial_std = 0.1
angular_theta_horizontal_std = 0.1
angular_phi_vertival_std = 0.1




''' Global variables '''
''' Heading of the uav '''
theta = 0.
''' Angle between uav heading and line joining uav and beacon '''
phi  = 0.
''' Radial distance between uav and beacon in vertical plane '''
R = 0.
''' Radial distance between uav and beacon in horizontal plane '''
r = 0.


''' Gazebo simulation messages '''
uav_pos = Point()
uav_rot = Point()
beacon_pos = Point()



''' Gazebo messages callback for beacon and uav positions'''
def models_cb(data):
    global  beacon_pos , uav0_pos, uav0_rot , theta, phi , R, r
    uav0_pos_x = data.pose.position.x
    uav0_pos_y = data.pose.position.y
    uav0_pos_z = data.pose.position.z
    uav0_rot = data.pose.orientation
    beacon_pos = data.pose.position
    orientation_q = data.pose.orientation
    orientation_list = (orientation_q.x, orientation_q.y,
                        orientation_q.z, orientation_q.w)
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
     ''' theta is '''
    theta = theta  + np.arctan2( beacon_pos.y - uav_pos_y ,  beacon_pos.x - uav_pos_x )
    ''' r is '''
    r = r + math.sqrt(math.pow(uav_pos_x - beacon_pos.x, 2) + math.pow(uav_pos_y - beacon_pos.y, 2))
    ''' phi is '''
    phi = phi + np.arctan2( beacon_pos.z - uav_pos_z ,  r )
    ''' R is '''
    R = R + math.sqrt(math.pow(uav_pos_x - beacon_pos.x, 2) + math.pow(uav_pos_y - beacon_pos.y, 2) + 
                                                             math.pow(uav_pos_z - beacon_pos.z, 2) )
    
    ''' You can print the beacon_pos if needed '''

def motion_model():
    G = np.array([0, np.dot(cos(phi),cos(theta)), -np.dot(R,np.dot(cos(phi),sin(theta))), -np.dot(R,np.dot(sin(phi),cos(theta)))], \
                 [0, np.dot(cos(phi),sin(theta)),  np.dot(R,np.dot(cos(phi),cos(theta))), -np.dot(R,np.dot(sin(phi),sin(theta)))],\
                 [0, sin(phi), 0, np.dot(R,cos(phi))]) 
    
    
    return G

 def measurement_model():
     H = np.array([(x-xm)/r, (y-ym)/r, 0], \
                  [(x-xm)/R, (y-ym)/R, (z-zm)/R], \
                  [-(y-ym)/(r**2), (x-xm)/(r**2), 0], \
                  [-(x-xm)/(np.dot(R**2,r)),-(y-ym)/(np.dot(R**2,r)), r/(R**2)]) 
   
    return H

def predict():
    global phi_old, theta_old, psi_old, u_old, v_old, w_old, x_old, y_old, z_old, dt, Sig, Qt
    global x_est, y_est, z_est, u_est, v_est, w_est, phi_est, theta_est, psi_est

    old_state = np.array([[x_old, y_old, z_old, u_old, v_old, w_old, phi_old, theta_old, psi_old]]).T
    
    est_state = old_state + est_motion*dt
    Sig = np.dot(motion_jac, np.dot(Sig, motion_jac.T)) + np.dot(inp_jac, np.dot(Qt, inp_jac.T))

    x_est = est_state[0, 0]
    y_est = est_state[1, 0]
    z_est = est_state[2, 0]

    
def correct():
    global H 
	# H = measurement_model().
    a = np.diag([0.0001,0.0001,0.0001])
    K = np.dot(Sig, np.dot(H.T, np.linalg.inv(a + np.dot(H, np.dot(Sig, H.T))) + Rt))
	# print(K)
    return K

def update():
    global phi_old, theta_old, psi_old, u_old, v_old, w_old, x_old, y_old, z_old, dt, Sig, Qt, Rt, H 
    global x_est, y_est, z_est, u_est, v_est, w_est, phi_est, theta_est, psi_est
    global x_new, y_new, z_new, u_new, v_new, w_new, phi_new, theta_new, psi_new, new_pos
	
    K = correct()
    #K[:, 2] = np.ones(9)
    #print(K)

    error = np.array([[x_gps - x_est, y_gps - y_est, z_gps - z_est]]).T
    #print(error)
    est_state = np.array([[x_est, y_est, z_est, u_est, v_est, w_est, phi_est, theta_est, psi_est]]).T

    new_state = est_state + np.dot(K, error)

    Sig = np.dot((np.eye(9) - np.dot(K, H)), Sig)

    x_new = new_state[0, 0]
    y_new = new_state[1, 0]
    z_new = new_state[2, 0]

    new_pos.x = x_new
    new_pos.y = y_new
    new_pos.z = z_new

rospy.init_node('beacon')
rate = rospy.Rate(100)
''' Subscribe to gazebo models and turtlebot odom '''
rospy.Subscriber('/gazebo/model_states', ModelStates, models_cb)
rospy.Subscriber('/uav0/mavros', PoseStamped, models_cb)
''' beacon signal publisher '''
beacon_pub = rospy.Publisher('beacon', ProjectMsg, queue_size=3)

print(" Beacon is at position (" + str(beacon_pos.x) + ", " + str(beacon_pos.y) )

''' Main loop '''
while not rospy.is_shutdown():
    ''' Compute distance between beacon and turtlebot '''
    distance = math.sqrt(math.pow(turtlebot_pos.x - beacon_pos.x,
                                  2) + math.pow(turtlebot_pos.y - beacon_pos.y, 2))
    ''' If turtlebot within beacon threshold '''
    if(distance < beacon_threshold):
        beaconMsg = BeaconMsg()
        beaconMsg.header.stamp = rospy.Time.now()
        ''' Adding noise to pure measurements '''
        beaconMsg.beacon_rho = np.random.normal(distance, radial_std)
        beaconMsg.beacon_phi = np.random.normal(phi, angular_std)
        ''' Publish rho and phi '''
        beacon_pub.publish(beaconMsg)

        rate.sleep()
