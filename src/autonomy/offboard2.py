import rospy
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, PositionTarget
from enum import Enum
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np
rospy.init_node('uav1_offboard')
rate = rospy.Rate(10)

current_state = None

# State machine for the drone.
class FSMSTATES(Enum):
    TAKEOFF = 3
    PERIODIC = 5
    INIT = 6

class UAV():
    fsmState = FSMSTATES.TAKEOFF
    last_req = rospy.Time.now()
    state  = State()
    theta = 0

uav1 = UAV()

def state_cb(msg):
    #print(msg)
    uav1.state = msg
    if uav1.fsmState == FSMSTATES.TAKEOFF :
        pass
    # if msg.armed == True and uav1.fsmState == FSMSTATES.ARMED:
    #     takeoff_uav()
    # if msg.mode == "AUTO.OFFBOARD":
    #     arm_uav()

def pos_sub(msg):

    position = np.array([
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z        
    ])
    if uav1.fsmState == FSMSTATES.TAKEOFF and np.linalg.norm( position - np.array([0,2,2]) ) < 0.1 :
        uav1.fsmState = FSMSTATES.PERIODIC
        print("Changing mode to PERIODIC")

state_sub = rospy.Subscriber("/uav1/mavros/state", State , state_cb)
local_pos_sub = rospy.Subscriber("/uav1/mavros/local_position/odom", Odometry, pos_sub)
local_pos_pub = rospy.Publisher("/uav1/mavros/setpoint_position/local", PoseStamped, queue_size= 10)
raw_pub = rospy.Publisher("/uav1/mavros/setpoint_raw/local", PositionTarget, queue_size= 10)
velocity_pub = rospy.Publisher("/uav1/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size= 10)
arming_client = rospy.ServiceProxy('/uav1/mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('/uav1/mavros/set_mode', SetMode)

def arm_uav():
    res = arming_client(True)
    if not res.success :
        print("Error arming the drone.")
    else:
        uav1.fsmState = FSMSTATES.ARMED
        print("UAV Armed")

def offboard_uav():
    res = set_mode_client(0, "OFFBOARD") # 0 is the base_mode? AUTO 
    if not res.mode_sent:
        print("Unable to send the mode")
    else:
        uav1.fsmState = FSMSTATES.OFFBOARD
        print("UAV command sent to set OFFBOARD")

def takeoff_uav():
    pose = PoseStamped()
    pose.pose.position.x = 2
    pose.pose.position.y = 2
    pose.pose.position.z = 2
    local_pos_pub.publish(pose)
    print("Command sent to perch at 2,2,2")

# Wait for connection from FCU
while(not rospy.is_shutdown() and not uav1.state.connected):
    pass

print("Connection Established")

pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 2
pose.pose.position.z = 2

setpoint_msg = TwistStamped(
   	    header=Header(
	    stamp=rospy.Time.now()),
)

# initialize the setpoint
setpoint_msg.twist.linear.x = 2
setpoint_msg.twist.angular.z = 0.1

# send 50 setpoints before starting
for i in range(0, 50):
    velocity_pub.publish(setpoint_msg)
    rate.sleep()

res  = arming_client(True)

if not res.success:
    print("Error ARMING the drone.")


def set_next_point():
    pose = PoseStamped()
    # pose.pose.position.x = radius * math.cos(theta)
    # pose.pose.position.y = radius * math.sin(theta)
    # pose.pose.position.z = height
    pose.pose.position.x = 0
    pose.pose.position.y = 2
    pose.pose.position.z = 2
    local_pos_pub.publish(pose)
    # theta = theta + 0.01
    # if theta > 2 * math.pi:
    #     theta = 0

def set_periodic_motion():
    # vel_cmd = TwistStamped()
    # vel_cmd.twist.linear.x = 2
    # vel_cmd.twist.angular.z = 0.1
    # velocity_pub.publish(vel_cmd)
    # vel_cmd = PositionTarget()
    # vel_cmd.velocity.x = 2
    # vel_cmd.yaw = 0.1
    # raw_pub.publish(vel_cmd)
    pose = PoseStamped()
    radius = 2
    height = 2
    pose.pose.position.x = radius * math.sin(uav1.theta)
    pose.pose.position.y = radius * math.cos(uav1.theta)
    pose.pose.position.z = height
    local_pos_pub.publish(pose)
    uav1.theta = uav1.theta + 0.01
    if uav1.theta > 2 * math.pi:
        uav1.theta = 0


while(not rospy.is_shutdown()):
    if ( not (uav1.state.mode == "OFFBOARD") and 
        ( (rospy.Time.now() - uav1.last_req) > rospy.Duration(5))) :
        set_mode_client.call(0, "OFFBOARD")
    elif ( not ( uav1.state.armed == True ) and
        ( (rospy.Time.now() - uav1.last_req) > rospy.Duration(5))) :
        arming_client.call(True)
    
    #velocity_pub.publish(setpoint_msg)
    #set_next_point()

    if ( uav1.fsmState == FSMSTATES.TAKEOFF ):
        set_next_point()
    if ( uav1.fsmState == FSMSTATES.PERIODIC ):
        set_periodic_motion()
    
    #set_vel()
    # if ( not uav1.state.mode == "OFFBOARD" and 
    #     ( (rospy.Time.now() - uav1.last_req) > rospy.Duration(1)) ):
    #     print("setting mode to OFFBOARD")
    #     res = set_mode_client.call(0, "OFFBOARD")
    #     if( res.mode_sent):
    #         print("OFFBOARD ENABLED")
    #     uav1.last_req = rospy.Time.now()
    # else:
    #     if ( not uav1.state.armed and
    #     (rospy.Time.now() - uav1.last_req > rospy.Duration(5.0))):
    #         res = arming_client(True)
    #         if res.success :
    #             print("UAV Armed")
    #     uav1.last_req = rospy.Time.now()
    rate.sleep()
