#include "ground_truth_pose.hh"

ground_truth_pose_bridge::ground_truth_pose_bridge()
{
  gtruth_odom_sub = n.subscribe("ground_truth/state", 10, &ground_truth_pose_bridge::ground_truth_odometry_callback, this);
  gtruth_pose_pub = n.advertise<geometry_msgs::PoseStamped>("ground_truth/pose", 10);;
}

ground_truth_pose_bridge::~ground_truth_pose_bridge()
{
}

void ground_truth_pose_bridge::ground_truth_odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  geometry_msgs::PoseWithCovariance my_pose_wCov;

  my_pose_wCov = msg->pose;
  pose_stamped.header = msg->header;
  pose_stamped.pose = my_pose_wCov.pose;

  gtruth_pose_pub.publish(pose_stamped);
  //ROS_INFO("x: %f", my_pose_wCov.pose.position.x);
}



int main(int argc, char** argv) {

  ros::init(argc, argv, "ground_truth_pose_bridge_node");

  ROS_INFO("Starting the node\n");
  ground_truth_pose_bridge my_bridge;
  ros::spin();
  //my_hardware_interface.spin();

  return 0;
}
