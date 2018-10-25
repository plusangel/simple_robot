#ifndef _GET_ODOM_PLUGIN_HH
#define _GET_ODOM_PLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

// ROS
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Boost
#include <boost/bind.hpp>

namespace gazebo
{
  class OdomPlugin : public ModelPlugin
  {
  public:
    OdomPlugin();

    ~OdomPlugin();

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  protected:
    virtual void UpdateChild();

    private:
      void PublishOdometry(double step_time);
      void UpdateOdometryEncoder();

      physics::WorldPtr world;
      physics::ModelPtr parent;
      event::ConnectionPtr updateConnection;

      double wheelSpeed[2];
      double odomPose[3];
      double odomVel[3];

      physics::JointPtr joints[2];
      physics::PhysicsEnginePtr physicsEngine;

      std::string frontLeftJointName;
      std::string frontRightJointName;
      std::string robotNamespace;

      double wheelSeparation;
      double wheelDiameter;
      int publish_tf;

      //time handling
      double update_rate;
      double update_period;
      common::Time last_update_time;
      common::Time last_odom_update;

      // ROS stuff
      ros::NodeHandle *rosnode_;
      ros::Publisher pub_odom;
      ros::Publisher pub_pose;

      nav_msgs::Odometry odom;
      geometry_msgs::Pose2D pose_;
      geometry_msgs::Pose2D pose_encoder;

      tf::TransformBroadcaster *transform_broadcaster_;
      std::string tf_prefix_;
  };
}

#endif
