#include "odom.hh"

using namespace gazebo;

enum
{
  RIGHT,
  LEFT,
};

OdomPlugin::OdomPlugin()
{
}

OdomPlugin::~OdomPlugin()
{
  delete rosnode_;
  delete transform_broadcaster_;
}

// Load the plugin
void OdomPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->parent = _parent;
  this->world = _parent->GetWorld();

  ROS_INFO("[odom]: plugin parent: %s", parent->GetName().c_str());

  if (!this->parent)
  {
    ROS_ERROR("[odom]: Plugin requires a Model as its parent");
  }

  this->frontLeftJointName = (_sdf->GetElement("frontLeftJoint")->Get<std::string>());
  ROS_INFO ("[odom]: frontLeftJointName: %s", this->frontLeftJointName.c_str());

  this->frontRightJointName = (_sdf->GetElement("frontRightJoint")->Get<std::string>());
  ROS_INFO ("[odom]: frontRightJointName: %s", this->frontRightJointName.c_str());

  this->wheelSeparation = atof((_sdf->GetElement("wheelSeparation")->Get<std::string>()).c_str());
  ROS_INFO ("[odom]: wheelSeparation: %f", this->wheelSeparation);

  this->wheelDiameter = atof((_sdf->GetElement("wheelDiameter")->Get<std::string>()).c_str());
  ROS_INFO ("[odom]: wheelDiameter: %f", this->wheelDiameter);

  this->robotNamespace = "";

  joints[LEFT] = this->parent->GetJoint(frontLeftJointName);
  joints[RIGHT] = this->parent->GetJoint(frontRightJointName);

  // ROS stuff
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "odom_plugin", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle(this->robotNamespace);

  ROS_INFO("[odom]: starting odom plugin in ns: %s", this->robotNamespace.c_str());

  tf_prefix_ = tf::getPrefixParam(*rosnode_);
  transform_broadcaster_ = new tf::TransformBroadcaster();

  pub_odom = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);
  pub_pose = rosnode_->advertise<geometry_msgs::Pose2D>("pose", 1);

  // Reset odometric pose
  odomPose[0] = 0.0;
  odomPose[1] = 0.0;
  odomPose[2] = 0.0;

  odomVel[0] = 0.0;
  odomVel[1] = 0.0;
  odomVel[2] = 0.0;

  // listen to the update event (broadcast every simulation iteration)
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&OdomPlugin::UpdateChild, this));
}

void OdomPlugin::UpdateChild()
{
  double ws, wd;
  double d1, d2;
  double dr, da;
  double stepTime = this->world->GetPhysicsEngine()->GetMaxStepSize();

  wd = wheelDiameter;
  ws = wheelSeparation;

  // Distance travelled from the front wheels
  d1 = stepTime * wd / 2 * joints[LEFT]->GetVelocity(0);
  d2 = stepTime * wd / 2 * joints[RIGHT]->GetVelocity(0);
  //ROS_INFO(">>d1: %f - d2: %f", d1, d2);

  dr = (d1 + d2) / 2;
  da = (d1 - d2) / ws;
  //ROS_INFO(">>dr: %f - da: %f", dr, da);

  // Compute odometric pose
  odomPose[0] += dr * cos(odomPose[2]);
  odomPose[1] += dr * sin(odomPose[2]);
  odomPose[2] += da;

  // Compute odometric instantaneous velocity
  odomVel[0] = dr / stepTime;
  odomVel[1] = 0.0;
  odomVel[2] = da / stepTime;

  publish_odometry();
}

void OdomPlugin::publish_odometry()
{
  ros::Time current_time = ros::Time::now();
  std::string odom_frame = tf::resolve(tf_prefix_, "odom");
  std::string base_footprint_frame = tf::resolve(tf_prefix_, "base_footprint");

  // getting data for base_footprint to odom transform
  gazebo::physics::ModelState state(this->parent);
  math::Pose pose = state.GetPose();

  tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
  tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

  tf::Transform base_footprint_to_odom(qt, vt);
  transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom, current_time, odom_frame, base_footprint_frame));

   // publish odom topic
   odom_.pose.pose.position.x = pose.pos.x;
   odom_.pose.pose.position.y = pose.pos.y;

   odom_.pose.pose.orientation.x = pose.rot.x;
   odom_.pose.pose.orientation.y = pose.rot.y;
   odom_.pose.pose.orientation.z = pose.rot.z;
   odom_.pose.pose.orientation.w = pose.rot.w;

   math::Vector3 linear = this->parent->GetWorldLinearVel();
   odom_.twist.twist.linear.x = linear.x;
   odom_.twist.twist.linear.y = linear.y;
   odom_.twist.twist.angular.z = this->parent->GetWorldAngularVel().z;

   odom_.header.stamp = current_time;
   odom_.header.frame_id = odom_frame;
   odom_.child_frame_id = base_footprint_frame;

   pub_odom.publish(odom_);


   double roll, pitch, yaw;
   tf::Matrix3x3(qt).getRPY(roll, pitch, yaw);
   //ROS_INFO("[odom]: Yaw: %f", yaw);

   pose_.x = pose.pos.x;
   pose_.y = pose.pos.y;
   pose_.theta = yaw;

   pub_pose.publish(pose_);
}

GZ_REGISTER_MODEL_PLUGIN(OdomPlugin)
