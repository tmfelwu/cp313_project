import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
rospy.init_node('uav0_offboard')
rate = rospy.Rate(100)

current_state = None

def state_cb(msg):
    global current_state
    current_state = msg
    print(msg)

state_sub = rospy.Subscriber("/uav0/mavros/state", State , state_cb)
local_pos_pub = rospy.Publisher("/uav0/mavros/setpoint_position/local", PoseStamped, queue_size= 10)
arming_client = rospy.ServiceProxy('/uav0/mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('/uav0/mavros/set_mode', SetMode)

pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 2

# for i in range(100):
#     local_pos_pub.publish(pose)
#     rospy.spin()
#     rate.sleep()

res  = arming_client(True)

if not res.success:
    print("ERROR")

res = set_mode_client(0, "OFFBOARD") # 0 is the base_mode? AUTO 

if not res.mode_sent:
    print(res)


while(not rospy.is_shutdown()):
    #set_mode_client.call(0, "OFFBOARD")
    local_pos_pub.publish(pose)
    #rospy.spin()
    rate.sleep()
