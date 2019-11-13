import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
import numpy as np
import filterpy
import time


''' ROS Node initiation '''
rospy.init_node('uav_pose_ekf_node')
rate = rospy.Rate(100)


'''global variables'''
global  Sig, Qt, Rt, H

global phi_old, theta_old, psi_old, phi_new, theta_new, psi_new, u_old, v_old, w_old, u_new, v_new, w_new, x_old, y_old, z_old, x_new, y_new, z_new, dt, time_now, time_prev

global ax, ay, az, p, q, r, x_gps, y_gps, z_gps, g

global x_est, y_est, z_est, u_est, v_est, w_est, phi_est, theta_est, psi_est

global x_gps, y_gps, z_gps, x_or, y_or, z_or

global new_pos 

global count 

Sig = 0.2*np.eye(9)
Qt = 0.2*np.eye(6)  
Rt = 0.2*np.eye(3)
H = np.hstack((np.eye(3), np.zeros((3, 6))))
g = 9.81
count = 0

phi_old = 0
theta_old = 0
psi_old = 0

u_old = 0
v_old = 0
w_old = 0

x_old = 0
y_old = 0
z_old = 0

time_now = time.time()
time_prev = time.time()

new_pos = Point()

''' Imu data callback'''
def imu_data_cb(data):
    # print('####    IMU    ########')
    # print(data)
    global ax, ay, az, p, q, r, dt, time_now, time_prev
    p = data.angular_velocity.x
    q = data.angular_velocity.y
    r = data.angular_velocity.z
    ax = data.linear_acceleration.x
    ay = data.linear_acceleration.y
    az = data.linear_acceleration.z
    time_now = time.time()
    dt = time_now - time_prev
    time_prev = time_now
    # print('------------------------\n')
    # update(x_est, y_est, z_est, x_imu, y_imu, z_imu, Sig,dt)
    # update(x_est, y_est, z_est, x_imu, y_imu, z_imu)
    predict()
    pass

''' GPS data callback '''
def gps_data_cb(data):
    # print('####    GPS   ####')
    # print(data)
    global x_gps, y_gps, z_gps, count, x_ori, y_ori, z_ori

    lat = data.latitude
    lon = data.longitude
    lat, lon = np.deg2rad(lat), np.deg2rad(lon)
    R = 6371000 # radius of the earth

    if count == 0:
        x_ori = R * np.cos(lat) * np.cos(lon)
        y_ori = R * np.cos(lat) * np.sin(lon)
        z_ori = R *np.sin(lat)
        count = count + 1
    else:
        x_gps = R * np.cos(lat) * np.cos(lon) - x_ori
        y_gps = R * np.cos(lat) * np.sin(lon) - y_ori
        z_gps = R *np.sin(lat) - z_ori
    # print(z_gps)
    # print('-----------------------\n')
    # return x_gps, y_gps #,z_gps
    update()

def alt_cb(data):
    #print('#####    Altitude     #####')
    #print(data.data)
    #print('------------------------\n')
    global z_gps
    z_gps = data.data
    # return data

''' Magnetometer  data callback'''
def mag_data_cb(data):
    # print('####     Magnetometer     #####')
    # print(data)
    x_mag = data.magnetic_field.x
    y_mag = data.magnetic_field.y
    # print('-------------------------\n')
    pass

def motion_model():
    C = np.array([[np.cos(theta_old)*np.cos(psi_old), np.cos(theta_old)*np.sin(psi_old), -np.sin(theta_old)], \
                
                [np.sin(theta_old)*np.sin(phi_old)*np.cos(psi_old) - np.sin(psi_old)*np.cos(phi_old), \
                np.sin(psi_old)*np.sin(theta_old)*np.sin(phi_old) + np.cos(psi_old)*np.cos(phi_old), np.sin(phi_old)*np.cos(theta_old)] , \
                
                [np.sin(theta_old)*np.cos(phi_old)*np.cos(psi_old) + np.sin(psi_old)*np.sin(phi_old), \
                np.sin(psi_old)*np.sin(theta_old)*np.cos(phi_old) - np.cos(psi_old)*np.sin(phi_old), np.sin(psi_old)*np.cos(theta_old)]])
    old_vel = np.array([[u_old, v_old, w_old]]).T
    est_vel = np.dot(C.T, old_vel)
    
    R = np.array([[1, np.sin(phi_old)*np.tan(theta_old), np.cos(phi_old)*np.tan(theta_old)], \
                [0, np.cos(phi_old), -np.sin(phi_old)], \
                [0, np.sin(phi_old)/np.cos(theta_old), np.cos(phi_old)/np.cos(theta_old)]])
    ang_vel = np.array([[p, q, r]]).T
    est_ang_vel = np.dot(R, ang_vel)

    u_dot = ax + v_old*r - w_old*q + g*np.sin(theta_old)
    v_dot = ay - u_old*r + w_old*p - g*np.cos(theta_old)*np.sin(phi_old)
    w_dot = az + u_old*q - v_old*p - g*np.cos(theta_old)*np.cos(phi_old)
    est_acc = np.array([[u_dot, v_dot, w_dot]]).T

    est_motion = np.vstack((est_vel, est_acc, est_ang_vel))

    row1 = np.array([[0, -np.sin(theta_old)*np.cos(psi_old)*u_old -np.sin(theta_old)*np.sin(psi_old)*v_old -np.cos(theta_old)*w_old, -np.cos(theta_old)*np.sin(psi_old)*u_old + np.cos(theta_old)*np.cos(psi_old)*v_old]])
    
    row2 = np.array([[u_old*(np.sin(theta_old)*np.cos(phi_old)*np.cos(psi_old) + np.sin(psi_old)*np.sin(phi_old)) + v_old*(np.sin(psi_old)*np.sin(theta_old)*np.cos(phi_old) - np.cos(psi_old)*np.sin(phi_old)) + w_old*(np.cos(phi_old)*np.cos(theta_old)), \
                    np.cos(theta_old)*np.sin(phi_old)*np.cos(psi_old)*u_old + np.sin(psi_old)*np.cos(theta_old)*np.sin(phi_old)*v_old - np.sin(phi_old)*np.sin(theta_old)*w_old, \
                    u_old*(-np.sin(theta_old)*np.sin(phi_old)*np.sin(psi_old) - np.cos(psi_old)*np.cos(phi_old)) + v_old*(np.cos(psi_old)*np.sin(theta_old)*np.sin(phi_old) - np.sin(psi_old)*np.cos(phi_old))]])
    
    row3 = np.array([[u_old*(-np.sin(theta_old)*np.sin(phi_old)*np.cos(psi_old) + np.sin(psi_old)*np.cos(phi_old)) + v_old*(-np.sin(psi_old)*np.sin(theta_old)*np.sin(phi_old) - np.cos(psi_old)*np.cos(phi_old)) - w_old*(np.sin(phi_old)*np.cos(theta_old)) -np.cos(phi_old)*np.sin(theta_old)*w_old, \
                    np.cos(theta_old)*np.cos(phi_old)*np.sin(psi_old)*u_old + np.sin(psi_old)*np.cos(theta_old)*np.cos(phi_old)*v_old, \
                    u_old*(-np.sin(theta_old)*np.cos(phi_old)*np.sin(psi_old) + np.cos(psi_old)*np.sin(phi_old)) + v_old*(np.cos(psi_old)*np.sin(theta_old)*np.cos(phi_old) + np.sin(psi_old)*np.sin(phi_old))]])
    
    row4 = np.array([[0, g*np.cos(theta_old), 0]])
    row5 = np.array([[-g*np.cos(theta_old)*np.cos(phi_old), g*np.sin(theta_old)*np.sin(phi_old), 0]])
    row6 = np.array([[0, g*np.sin(theta_old)*np.cos(phi_old), 0]])
    row7 = np.array([[np.cos(phi_old)*np.tan(theta_old)*q - np.sin(phi_old)*np.tan(theta_old)*r, (np.sin(phi_old)/np.cos(theta_old)**2)*q + (np.cos(phi_old)/np.cos(theta_old)**2)*r, 0]])
    row8 = np.array([[-np.sin(phi_old)*q - np.cos(phi_old)*r, 0, 0]])
    row9 = np.array([[(np.cos(phi_old)/np.cos(theta_old))*q - (np.sin(phi_old)/np.cos(theta_old))*r, (np.sin(phi_old)/np.cos(theta_old))*np.tan(theta_old)*q + (np.cos(phi_old)/np.cos(theta_old))*np.tan(theta_old)*r, 0]])

    a = np.zeros((9, 3))
    w_cross = np.array([[0, -r, q], [r, 0, -p], [-q, p, 0]])
    b = np.zeros((3, 3))
    motion_jac = np.hstack((a, np.vstack((C, w_cross, b)), np.vstack((row1, row2, row3, row4, row5, row6, row7, row8, row9))))
    v_cross = np.array([[0, -w_old, v_old], [w_old, 0, -u_old], [-v_old, u_old, 0]])
    inp_jac = np.hstack((np.vstack((b, np.eye(3), b)), np.vstack((b, v_cross.T, R.T))))
    
    return est_motion, motion_jac, inp_jac

# def measurement_model():
#    return np.hstack((np.eye(3), np.zeros((3, 6))))

def predict():
    global phi_old, theta_old, psi_old, u_old, v_old, w_old, x_old, y_old, z_old, dt, Sig, Qt
    global x_est, y_est, z_est, u_est, v_est, w_est, phi_est, theta_est, psi_est

    old_state = np.array([[x_old, y_old, z_old, u_old, v_old, w_old, phi_old, theta_old, psi_old]]).T
    est_motion, motion_jac, inp_jac = motion_model()
    est_state = old_state + est_motion*dt
    Sig = np.dot(motion_jac, np.dot(Sig, motion_jac.T)) + np.dot(inp_jac, np.dot(Qt, inp_jac.T))

    x_est = est_state[0, 0]
    y_est = est_state[1, 0]
    z_est = est_state[2, 0]

    u_est = est_state[3, 0]
    v_est = est_state[4, 0]
    w_est = est_state[5, 0]

    phi_est = est_state[6, 0]
    theta_est = est_state[7, 0]
    psi_est = est_state[8, 0]
    
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

    u_new = new_state[3, 0]
    v_new = new_state[4, 0]
    w_new = new_state[5, 0]

    phi_new = new_state[6, 0]
    theta_new = new_state[7, 0]
    psi_new = new_state[8, 0]

    new_pos.x = x_new
    new_pos.y = y_new
    new_pos.z = z_new

''' Subscribers'''
rospy.Subscriber('/uav0/mavros/imu/data', Imu, imu_data_cb)
rospy.Subscriber('/uav0/mavros/global_position/raw/fix', NavSatFix, gps_data_cb)
rospy.Subscriber('/uav0/mavros/imu/mag', MagneticField, mag_data_cb)
# rospy.Subscriber('/uav1/mavros/global_position/rel_alt', Float64, alt_cb)

''' Publishers '''
#ground_truth_pub = rospy.Publisher('/ground_truth_yaw', Point, queue_size=1)
estimated_pos_pub = rospy.Publisher('/est_pos', Point, queue_size=3)

while not rospy.is_shutdown():
   estimated_pos_pub.publish(new_pos)
   print("x = {:.2f}, y = {:.2f}, z = {:.2f}\n".format(new_pos.x, new_pos.y, new_pos.z))

   rate.sleep()
