from tutorial_pkg.srv import MyAddTwoInts
from tutorial_pkg.srv import MyAddTwoIntsRequest
from tutorial_pkg.srv import MyAddTwoIntsResponse


import rospy

def handle_addition(req):
    return MyAddTwoIntsResponse(req.a + req.b)

def add():
    rospy.init_node('adder')
    s = rospy.Service('add_two_int64s', MyAddTwoInts, handle_addition)
    print("Ready to add")
    rospy.spin()

if __name__ == "__main__":
    add()