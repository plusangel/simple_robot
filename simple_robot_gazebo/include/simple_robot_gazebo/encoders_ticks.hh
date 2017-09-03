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
    // Constactor
    public: EncodersTicksPlugin();

    // Destructor
    public: virtual ~EncodersTicksPlugin();

    //  Load the sensor plugin.
    public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/);

    // Pointer to the model
    private: physics::ModelPtr model;

    // List which holds the innoclimber joints
    private: physics::Joint_V jointList;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS NodeHanle
    private: ros::NodeHandle* rosNode;
    private: ros::Publisher encoders_pub;
  };
}
#endif
