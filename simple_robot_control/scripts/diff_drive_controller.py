#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class ControllerNode:

    def __init__(self):
        self.linearVelocity = 0.0
        self.angularVelocity = 0.0


    def publish(self):
        if rospy.get_time() - self.lastTwistTime < self.timeout:

            self.right = 1.0 * self.linearVelocity + self.angularVelocity * self.baseWidth / 2
            self.left = 1.0 * self.linearVelocity - self.angularVelocity * self.baseWidth / 2

            rospy.loginfo("Sending velocities to wheels... vl:{0}, vr:{1}".format(self.left, self.right))

            self.leftPub.publish(self.left)
            self.rightPub.publish(self.right)
        else:
            self.leftPub.publish(0)
            self.rightPub.publish(0)


    def twistCallback(self, twist):
        self.linearVelocity = twist.linear.x
        self.angularVelocity = twist.angular.z
        self.lastTwistTime = rospy.get_time()

        #rospy.loginfo("linear: {0}, angular: {1}, time: {2}".format(self.linearVelocity, \
        #self.angularVelocity, self.lastTwistTime))

    def main(self):

        # Create the publishers
        self.leftPub = rospy.Publisher('lWheels', Float32, queue_size=10)
        self.rightPub = rospy.Publisher('rWheels', Float32, queue_size=10)

        # Create the coresponding node
        rospy.init_node('diff_drive_controller')
        self.nodeName = rospy.get_name()

        rospy.loginfo("{0} started".format(self.nodeName))

        # Listen to the classic topic for move commands
        rospy.Subscriber("cmd_vel", Twist, self.twistCallback)

        # Load the configuration file
        self.baseWidth = float(rospy.get_param('~base_width', 0.2))
        self.rate = float(rospy.get_param('~rate', 20.0))
        self.timeout = float(rospy.get_param('~timeout', 0.2))


        # Loop now!
        rate = rospy.Rate(self.rate)

        self.lastTwistTime = rospy.get_time()
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    try:
        node = ControllerNode()
        node.main()
    except rospy.ROSInterruptException:
        pass
