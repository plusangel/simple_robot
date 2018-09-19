#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

roll = pitch = yaw = 0.0

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    rospy.loginfo(np.rad2deg(yaw))

rospy.init_node('my_quaternion_to_euler')

sub = rospy.Subscriber ('/imu_data', Imu, get_rotation)

r = rospy.Rate(10)

while not rospy.is_shutdown():
    quat = quaternion_from_euler (roll, pitch,yaw)
    r.sleep()
