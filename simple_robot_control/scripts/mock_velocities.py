#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def mock_velocoties():
    pub_left_vel = rospy.Publisher('lWheels', Int32, queue_size=5)
    pub_right_vel = rospy.Publisher('rWheels', Int32, queue_size=5)

    rospy.init_node('mock_velocities', anonymous=True)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        vl = Int32(1)
        vr = Int32(1)
        #rospy.loginfo("Publishing mock velocities to topics...")
        pub_left_vel.publish(vl)
        pub_right_vel.publish(vr)
        rate.sleep()


if __name__ == '__main__':
    try:
        mock_velocoties()
    except:
        pass
