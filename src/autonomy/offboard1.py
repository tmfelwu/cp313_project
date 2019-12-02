import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool, SetMode
from enum import Enum
import numpy as np
rospy.init_node('uav0_offboard')
rate = rospy.Rate(10)

current_state = None

class FSMSTATES(Enum):
    TAKEOFF = 1
    PREDICT = 2
    INTERCEPT = 3

class UAV():
    fsmState = FSMSTATES.TAKEOFF
    last_req = rospy.Time.now()
    state = State()
    position = np.array([0,0,0])

uav0 = UAV()

def state_cb(msg):
    uav0.state = msg
    global current_state
    current_state = msg
    print(msg)

def pos_sub(msg):

    position = np.array([
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z        
    ])
    uav0.position = position
    if uav0.fsmState == FSMSTATES.TAKEOFF and np.linalg.norm( position - np.array([0,0,8]) ) < 0.1 :
        uav0.fsmState = FSMSTATES.PREDICT
        print("Changing mode to PREDICT, Starting taking images")



def rogue_cb(msg):

    x_uav_c = np.array([
        msg.x,
        msg.y,
        msg.z,
        1
    ])
    # Transform UAV to global
    R_gu = np.array([
        [1,0,0,uav0.position[0]],
        [0,1,0,uav0.position[1]],
        [0,0,1,uav0.position[2]],
        [0,0,0,1]
    ])

    x_world = np.dot(R_gu,x_uav_c)
    print(x_world)

state_sub = rospy.Subscriber("/uav0/mavros/state", State , state_cb)
local_pos_sub = rospy.Subscriber("/uav0/mavros/local_position/odom", Odometry, pos_sub)
local_pos_pub = rospy.Publisher("/uav0/mavros/setpoint_position/local", PoseStamped, queue_size= 10)
arming_client = rospy.ServiceProxy('/uav0/mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('/uav0/mavros/set_mode', SetMode)
rogue_sub = rospy.Subscriber("/rogue", Point, rogue_cb)
pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 8

# for i in range(100):
#     local_pos_pub.publish(pose)
#     rospy.spin()
#     rate.sleep()

# Wait for FCU connection 
# As long as it not connected let it sleep
while(not rospy.is_shutdown() and (not current_state == None and not current_state.connected) ):
    print("Waiting for connection")
    rospy.spin()
    rate.sleep()

res  = arming_client(True)

if not res.success:
    print("ERROR")

res = set_mode_client(0, "OFFBOARD") # 0 is the base_mode? AUTO 

if not res.mode_sent:
    print(res)


while(not rospy.is_shutdown()):

    if ( uav0.state.mode != "OFFBOARD" and ( rospy.Time.now() - uav0.last_req ) > rospy.Duration(5.0) ):
        print("LOG : Setting mode to OFFBOARD.")
        set_mode_client.call(0, "OFFBOARD")
        uav0.last_req = rospy.Time.now()
    elif  ( not ( uav0.state.armed == True ) and
        ( (rospy.Time.now() - uav0.last_req) > rospy.Duration(5))) :
        arming_client.call(True)

    if ( uav0.fsmState == FSMSTATES.TAKEOFF):
        local_pos_pub.publish(pose)

    if ( uav0.fsmState == FSMSTATES.PREDICT):
        local_pos_pub.publish(pose)
        pass
    
    if ( uav0.fsmState == FSMSTATES.INTERCEPT):
        pass
    
    rate.sleep()