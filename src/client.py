import sys
import rospy
import tutorial_pkg.srv import MyAddTwoInts

service = rospy.ServiceProxy('add_two_int64s', MyAddTwoInts)
ans = service(3,4)

print(ans)