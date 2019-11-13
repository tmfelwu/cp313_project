import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
rospy.init_node('uav0_offboard')
rate = rospy.Rate(10)

current_state = None
class Drone():
    def __init__(self):
        self.state = State()
        self.last_req = rospy.Time.now()
    
uav = Drone()

def state_cb(msg):
    global current_state
    current_state = msg
    uav.state = msg
    print(msg)


state_sub = rospy.Subscriber("/uav0/mavros/state", State , state_cb)
local_pos_pub = rospy.Publisher("/uav0/mavros/setpoint_position/local", PoseStamped, queue_size= 10)
arming_client = rospy.ServiceProxy('/uav0/mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('/uav0/mavros/set_mode', SetMode)

pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 6

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

    if ( uav.state.mode != "OFFBOARD" and ( rospy.Time.now() - uav.last_req ) > rospy.Duration(5.0) ):
        print("LOG : Setting mode to OFFBOARD.")
        set_mode_client.call(0, "OFFBOARD")
        uav.last_req = rospy.Time.now()

    local_pos_pub.publish(pose)
    rate.sleep()
