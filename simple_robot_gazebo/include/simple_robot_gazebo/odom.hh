#ifndef _GET_ODOM_PLUGIN_HH
#define _GET_ODOM_PLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

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
      void publish_odometry();

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

      // ROS stuff
      ros::NodeHandle *rosnode_;
      ros::Publisher pub_odom;
      ros::Publisher pub_pose;

      nav_msgs::Odometry odom_;
      geometry_msgs::Pose2D pose_;

      tf::TransformBroadcaster *transform_broadcaster_;
      std::string tf_prefix_;
  };
}

#endif
