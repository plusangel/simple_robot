#ifndef _ENCODERS_TICKS_PLUGIN_HH_
#define _ENCODERS_TICKS_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <cmath>
#include <simple_robot_gazebo/encoders.h>

namespace gazebo
{
  class EncodersTicksPlugin : public ModelPlugin
  {
  public:
    // Constactor
    EncodersTicksPlugin();

    // Destructor
    virtual ~EncodersTicksPlugin();

    //  Load the sensor plugin.
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/);

  private:
    // Pointer to the model
    physics::ModelPtr model;

    // List which holds the innoclimber joints
    physics::Joint_V jointList;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // ROS NodeHanle
    ros::NodeHandle* rosNode;
    ros::Publisher encoders_pub;

    // encoders resolution
    int encoders_resolution;

    // stddev between left and right side
    float left_stddev;
    float right_stddev;

    // normal distribution generator
    std::default_random_engine generator;
  };
}
#endif
