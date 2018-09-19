#!/usr/bin/env python

from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from geometry_msgs.msg import Twist
import time
import rospy

def move_server(req):
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10)
    twist_object = Twist()
    twist_object.linear.x = 0.5

    rospy.loginfo("service started")
    start = time.time()
    while time.time() - start < 10:
        cmd_vel_pub.publish(twist_object)
        rate.sleep()

    res = TriggerResponse()
    res.success = True
    res.message = "everything is cool"

    rospy.loginfo("service ended")
    return res

def move_robot():
    rospy.init_node('move_robot_server')

    s = rospy.Service('move_robot', Trigger, move_server)
    print "Ready to move the robot."
    rospy.spin()

if __name__ == "__main__":
    move_robot()
