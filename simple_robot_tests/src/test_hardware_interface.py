#!/usr/bin/env python

import rospy
import sys

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension

class motion_cmd_test:
    def __init__(self, arg, rot):

        rospy.on_shutdown(self.shutdown)

        self.pub = rospy.Publisher('joint_velocities', Float32MultiArray, queue_size=5)

        rospy.init_node('joint_velocities', anonymous=True)
        self.rate = rospy.Rate(20)
        self.wheels = arg

        if rot:
            r = -1
        else:
            r = 1

        # compose the multiarray message
        self.jointVelocities = Float32MultiArray()
        myLayout = MultiArrayLayout()
        myMultiArrayDimension = MultiArrayDimension()

        myMultiArrayDimension.label = "joint_velocities"
        myMultiArrayDimension.size = 1
        myMultiArrayDimension.stride = numOfWheels

        myLayout.dim = [myMultiArrayDimension]
        myLayout.data_offset = 0
        self.jointVelocities.layout = myLayout

        while not rospy.is_shutdown():
            # first item is left and second is right
            if self.wheels == 2:
                self.jointVelocities.data = [1.0, 1.0*r]
            elif self.wheels == 4:
                self.jointVelocities.data = [1.0, 1.0, 1.0*r, 1.0*r]

            self.pub.publish(self.jointVelocities)
            self.rate.sleep()

    def shutdown(self):

        try:
            rospy.loginfo("Stopping the robot...")

            if self.wheels == 2:
                self.jointVelocities.data = [0, 0]
            elif self.wheels == 4:
                self.jointVelocities.data = [0, 0, 0, 0]

            # publish the message and log in terminal
            self.pub.publish(self.jointVelocities)
            # rospy.loginfo("I'm publishing: [%d, %d]" % (self.pwmVelocities.data[0], self.pwmVelocities.data[1]))
            rospy.sleep(2)
        except:
            rospy.loginfo("Cannot stop!")


if __name__ == '__main__':
    numOfWheels = int(sys.argv[1])
    rotate  = int(sys.argv[2])
    try:
        my_command = motion_cmd_test(numOfWheels, rotate)
    except rospy.ROSInterruptException:
        pass
