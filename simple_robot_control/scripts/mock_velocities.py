#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension

def mock_velocoties():
    pub = rospy.Publisher('joint_velocities', Float32MultiArray, queue_size=5)

    rospy.init_node('joint_velocities', anonymous=True)
    rate = rospy.Rate(20)

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

    while not rospy.is_shutdown():
        # first item is left and second is right
        jointVelocities.data = [-1.0, 0.0]

        pub.publish(jointVelocities)
        rate.sleep()

if __name__ == '__main__':
    mock_velocoties()
