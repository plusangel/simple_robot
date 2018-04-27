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

  this->update_rate = atof((_sdf->GetElement("updateRate")->Get<std::string>()).c_str());
  ROS_INFO ("[odom]: rate: %f", this->update_rate);

  this->robotNamespace = "";

  joints[LEFT] = this->parent->GetJoint(frontLeftJointName);
  joints[RIGHT] = this->parent->GetJoint(frontRightJointName);

  // ROS stuff
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "odom_plugin", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle(this->robotNamespace);

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

  if (update_rate > 0.0)
    update_period = 1.0 / update_rate;
  else
    update_period = 0.0;

  last_update_time = parent->GetWorld()->GetSimTime();

  // listen to the update event (broadcast every simulation iteration)
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&OdomPlugin::UpdateChild, this));
}

void OdomPlugin::UpdateChild()
{
  UpdateOdometryEncoder();

  common::Time current_time = parent->GetWorld()->GetSimTime();
  double seconds_since_last_update = (current_time - last_update_time).Double();

  if (seconds_since_last_update > update_period) {
    PublishOdometry(seconds_since_last_update);

    last_update_time += common::Time(update_period);
  }
}

void OdomPlugin::PublishOdometry(double step_time)
{

  ros::Time current_time = ros::Time::now();
  std::string odom_frame = tf::resolve(tf_prefix_, "odom");
  std::string base_footprint_frame = tf::resolve(tf_prefix_, "base_footprint");

  tf::Quaternion qt;
  tf::Vector3 vt;

  qt = tf::Quaternion ( odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w );
  vt = tf::Vector3 ( odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z );


  tf::Transform base_footprint_to_odom ( qt, vt );
    transform_broadcaster_->sendTransform (
        tf::StampedTransform ( base_footprint_to_odom, current_time,
                               odom_frame, base_footprint_frame ) );

    // set header
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame;
    odom.child_frame_id = base_footprint_frame;

    // set pose covariance
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[14] = 1000000.0;
    odom.pose.covariance[21] = 1000000.0;
    odom.pose.covariance[28] = 1000000.0;
    odom.pose.covariance[35] = 0.03;

    // set twist covariance
    odom.twist.covariance[0] = 0.001;
    odom.twist.covariance[7] = 0.001;
    odom.twist.covariance[14] = 1000000.0;
    odom.twist.covariance[21] = 1000000.0;
    odom.twist.covariance[28] = 1000000.0;
    odom.twist.covariance[35] = 0.03;

    pub_odom.publish (odom);
}

void OdomPlugin::UpdateOdometryEncoder()
{
    double vl = joints[LEFT]->GetVelocity ( 0 );
    double vr = joints[RIGHT]->GetVelocity ( 0 );

    common::Time current_time = parent->GetWorld()->GetSimTime();

    double seconds_since_last_update = ( current_time - last_odom_update ).Double();
    last_odom_update = current_time;

    double b = wheelSeparation;

    // Book: Sigwart 2011 Autonompus Mobile Robots page:337
    double sl = vl * ( wheelDiameter / 2.0 ) * seconds_since_last_update;
    double sr = vr * ( wheelDiameter / 2.0 ) * seconds_since_last_update;
    double ssum = sl + sr;

    double sdiff;
    sdiff = sl - sr;
    //sdiff = sr - sl;

    double dx = ( ssum ) /2.0 * cos ( pose_encoder.theta + ( sdiff ) / ( 2.0*b ) );
    double dy = ( ssum ) /2.0 * sin ( pose_encoder.theta + ( sdiff ) / ( 2.0*b ) );
    double dtheta = ( sdiff ) /b;

    pose_encoder.x += dx;
    pose_encoder.y += dy;
    pose_encoder.theta += dtheta;

    double w = dtheta/seconds_since_last_update;
    double v = sqrt ( dx*dx+dy*dy ) /seconds_since_last_update;

    tf::Quaternion qt;
    tf::Vector3 vt;
    qt.setRPY ( 0,0,pose_encoder.theta );
    vt = tf::Vector3 ( pose_encoder.x, pose_encoder.y, 0 );

    odom.pose.pose.position.x = vt.x();
    odom.pose.pose.position.y = vt.y();
    odom.pose.pose.position.z = vt.z();

    odom.pose.pose.orientation.x = qt.x();
    odom.pose.pose.orientation.y = qt.y();
    odom.pose.pose.orientation.z = qt.z();
    odom.pose.pose.orientation.w = qt.w();

    odom.twist.twist.linear.x = dx/seconds_since_last_update;
    odom.twist.twist.linear.y = dy/seconds_since_last_update;
}

GZ_REGISTER_MODEL_PLUGIN(OdomPlugin)
