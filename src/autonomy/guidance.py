import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import math as m
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import NavSatFix
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64
from purepursuit import model
import rospy
import time

rospy.init_node('guidance')

global x_obs, y_obs , count, x_ori, y_ori, z_ori ,z_obs, x_rog, y_rog , x_ori, y_ori, z_rog 
rog_data = []
obs_data = []
x_ori = 0
y_ori = 0

count = 0

def gps_obs_cb(data):
    # print('####    GPS   ####')
    # print(data)
    global x_obs, y_obs , count, x_ori, y_ori, z_ori 

    lat = data.latitude
    lon = data.longitude
    lat, lon = np.deg2rad(lat), np.deg2rad(lon)
    R = 6371000 # radius of the earth

    if count == 0:
        x_ori = R * np.cos(lat) * np.cos(lon)
        y_ori = R * np.cos(lat) * np.sin(lon)
        z_ori = R *np.sin(lat)
        count = 1
    else:
        x_obs = R * np.cos(lat) * np.cos(lon) - x_ori
        y_obs = R * np.cos(lat) * np.sin(lon) - y_ori
    #print(x_obs, y_obs )

      

def obs_cb(data):
    #print('#####    Altitude     #####')
    #print(data.data)
    #print('------------------------\n')
    global z_obs , obs_data
    z_obs = data.data
    obs_data.append(['x_obs','y_obs','z_obs')

def gps_rog_cb(data):
    # print('####    GPS   ####')
    # print(data)
    global x_rog, y_rog , x_ori, y_ori

    lat = data.latitude
    lon = data.longitude
    lat, lon = np.deg2rad(lat), np.deg2rad(lon)
    R = 6371000 # radius of the earth
	
    x_rog = R * np.cos(lat) * np.cos(lon) - x_ori
    y_rog = R * np.cos(lat) * np.sin(lon) - y_ori
    print(x_rog, y_rog )


def rog_cb(data):
    #print('#####    Altitude     #####')
    #print(data.data)
    #print('------------------------\n')
    global z_rog,rog_data
    z_rog = data.data
    #print(z_rog )
   # rog_data.append('x_rog','y_rog','z_rog')

def pos_cb():

    global x_obs, y_obs ,  x_ori, y_ori, z_ori ,z_obs, x_rog, y_rog , x_ori, y_ori, z_rog  ,rog_data, obs_data ,alpha_obs,alpha_rog,los_range,los_rate 

    obs_data = obs_data[-2:]
    rog_data = rog_data[-2:]
    x_obs = obs_data[-1][0]
    y_obs = obs_data[-1][1]
    x_rog = rog_data[-1][0]
    y_rog = rog_data[-1][0]
    alpha_obs = m.atan((y_obs - obs_data[-2][1])/(x_obs - obs_data[-2][0]))
    alpha_rog = m.atan((y_rog - rog_data[-2][1])/(x_rog - rog_data[-2][0]))
    los_range = np.sqrt((x_obs - x_rog)**2 + (y_obs - y_rog)**2 + (z_obs - z_rog)**2)  
    los_rate = m.atan((y_obs - y_rog)/(x_obs - x_rog) ) 

    return x_obs,y_obs,x_rog,y_rog,alpha_obs,alpha_rog,los_range,los_range






rospy.Subscriber('/uav0/mavros/global_position/raw/fix', NavSatFix, gps_obs_cb)
rospy.Subscriber('/uav0/mavros/global_position/rel_alt', Float64, obs_cb)
rospy.Subscriber('/uav1/mavros/global_position/raw/fix', NavSatFix, gps_rog_cb)
rospy.Subscriber('/uav1/mavros/global_position/rel_alt', Float64, rog_cb)

obs_pos_pub = rospy.Publisher("/uav0/mavros/setpoint_position/local", PoseStamped, queue_size= 10)
pose = PoseStamped()


rate = rospy.Rate(10)

time.sleep(1)

while not rospy.is_shutdown():

    global x_obs, y_obs ,  x_ori, y_ori, z_ori ,z_obs, x_rog, y_rog , x_ori, y_ori, z_rog  ,rog_data, obs_data,alpha_obs,alpha_rog,los_range,los_rate 

    #pose.pose.position.z = z_obs 
    pose.pose.position.x = x_obs
    pose.pose.position.y = y_obs
   
    if z_obs>z_rog:
        pose.pose.position.z = 2.0
    #pose.pose.position.x = 5.0
    #print("x = {:.2f}, y = {:.2f}, z = {:.2f}\n".format(z_rog))
    obs_pos_pub.publish(pose)

    rate.sleep()