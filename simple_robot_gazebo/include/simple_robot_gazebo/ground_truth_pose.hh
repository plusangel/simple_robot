#ifndef _GROUND_TRUTH_POSE_HH_
#define _GROUND_TRUTH_POSE_HH_

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseStamped.h"

class ground_truth_pose_bridge {
public:
  // Constructor
  ground_truth_pose_bridge();

  // Destructor
  virtual ~ground_truth_pose_bridge();

  ros::Subscriber ground_truth_odom_listener;

private:

  ros::NodeHandle n;
  ros::Subscriber gtruth_odom_sub;
  ros::Publisher gtruth_pose_pub;

  geometry_msgs::PoseStamped pose_stamped;

  // callbck from groud truth odometry gazebo plugin
  void ground_truth_odometry_callback(const nav_msgs::Odometry::ConstPtr& );
};

#endif
