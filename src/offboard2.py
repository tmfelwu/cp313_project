import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
rospy.init_node('move_drone')
rate = rospy.Rate(100)

current_state = None

def state_cb(msg):
    global current_state
    current_state = msg
    print(msg)

state_sub = rospy.Subscriber("/uav1/mavros/state", State , state_cb)
local_pos_pub = rospy.Publisher("/uav1/mavros/setpoint_position/local", PoseStamped, queue_size= 10)
arming_client = rospy.ServiceProxy('/uav1/mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('/uav1/mavros/set_mode', SetMode)

pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 2

center = (0,0)
radius = 3
height = 2

# for i in range(100):
#     local_pos_pub.publish(pose)
#     rospy.spin()
#     rate.sleep()

res  = arming_client(True)

if not res.success:
    print("ERrro")

res = set_mode_client(0, "OFFBOARD") # 0 is the base_mode? AUTO 

if not res.mode_sent:
    print(res)

theta = 0
def set_next_point():
    global theta
    pose = PoseStamped()
    pose.pose.position.x = radius * math.cos(theta)
    pose.pose.position.y = radius * math.sin(theta)
    pose.pose.position.z = height
    local_pos_pub.publish(pose)
    theta = theta + 0.01
    if theta > 2 * math.pi:
        theta = 0

while(not rospy.is_shutdown()):
    set_mode_client.call(0, "OFFBOARD")
    set_next_point()
    #local_pos_pub.publish(pose)
    #rospy.spin()
    rate.sleep()
