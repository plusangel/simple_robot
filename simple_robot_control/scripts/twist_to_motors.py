#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension

class Twist_To_Motors:

    def __init__(self):
        self.linearVelocity = 0.0
        self.angularVelocity = 0.0


    def publish(self):

        # compose the multiarray message
        jointVelocities = Float32MultiArray()
        myLayout = MultiArrayLayout()
        myMultiArrayDimension = MultiArrayDimension()

        myMultiArrayDimension.label = "joint_velocities"
        myMultiArrayDimension.size = 1
        myMultiArrayDimension.stride = 2

        myLayout.dim = [myMultiArrayDimension]
        myLayout.data_offset = 0
        jointVelocities.layout = myLayout

        if rospy.get_time() - self.lastTwistTime < self.timeout:

            self.right = 1.0 * self.linearVelocity + self.angularVelocity * self.baseWidth / 2
            self.left = 1.0 * self.linearVelocity - self.angularVelocity * self.baseWidth / 2

            rospy.loginfo("Sending velocities to wheels... vl:{0}, vr:{1}".format(self.left, self.right))

            # first item is left and second is right
            jointVelocities.data = [self.left, self.right]
            self.joint_velocities_Pub.publish(jointVelocities)
        else:
            # first item is left and second is right
            jointVelocities.data = [0.0, 0.0]
            self.joint_velocities_Pub.publish(jointVelocities)



    def twistCallback(self, twist):
        self.linearVelocity = twist.linear.x
        self.angularVelocity = twist.angular.z
        self.lastTwistTime = rospy.get_time()

        #rospy.loginfo("linear: {0}, angular: {1}, time: {2}".format(self.linearVelocity, \
        #self.angularVelocity, self.lastTwistTime))

    def main(self):

        # Create the publishers
        self.joint_velocities_Pub = rospy.Publisher('joint_velocities', Float32MultiArray, queue_size=10)

        # Create the coresponding node
        rospy.init_node('twist_to_motors')
        self.nodeName = rospy.get_name()

        rospy.loginfo("{0} started".format(self.nodeName))

        # Listen to the classic topic for move commands
        rospy.Subscriber("cmd_vel", Twist, self.twistCallback)

        # Load the configuration file
        self.baseWidth = float(rospy.get_param('~base_width', 0.2))
        self.rate = float(rospy.get_param('~rate', 20.0))
        self.timeout = float(rospy.get_param('~timeout', 0.2))

        # Loop now
        rate = rospy.Rate(self.rate)

        self.lastTwistTime = rospy.get_time()
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = Twist_To_Motors()
        node.main()
    except rospy.ROSInterruptException:
        pass
