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
        rospy.logdebug(self._odomdata)

    def get_odomdata(self):
        """
        Returns the newest odom data

        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        string child_frame_id
        geometry_msgs/PoseWithCovariance pose
          geometry_msgs/Pose pose
            geometry_msgs/Point position
              float64 x
              float64 y
              float64 z
            geometry_msgs/Quaternion orientation
              float64 x
              float64 y
              float64 z
              float64 w
          float64[36] covariance
        geometry_msgs/TwistWithCovariance twist
          geometry_msgs/Twist twist
            geometry_msgs/Vector3 linear
              float64 x
              float64 y
              float64 z
            geometry_msgs/Vector3 angular
              float64 x
              float64 y
              float64 z
          float64[36] covariance

        """
        return self._odomdata

if __name__ == "__main__":
    rospy.init_node('odom_topic_subscriber', log_level=rospy.INFO)
    odom_reader_object = OdomTopicReader()
    rospy.loginfo(odom_reader_object.get_odomdata())
    rate = rospy.Rate(0.5)

    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        global ctrl_c
        print "shutdown time!"
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        data = odom_reader_object.get_odomdata()
        rospy.loginfo(data)
        rate.sleep()
