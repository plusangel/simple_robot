#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

class OdomTopicReader(object):
    def __init__(self, topic_name = '/odom'):
        self._topic_name = topic_name
        self._sub = rospy.Subscriber(self._topic_name, Odometry, self.topic_callback)
        self._odomdata = Odometry()

    def topic_callback(self, msg):
        self._odomdata = msg
        rospy.loginfo(self._odomdata)

if __name__ == "__main__":
    rospy.init_node('odom_topic_subscriber')
    odom_reader_object = OdomTopicReader()
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
