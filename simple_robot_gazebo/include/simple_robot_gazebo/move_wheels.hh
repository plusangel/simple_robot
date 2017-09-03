#ifndef _MOVE_WHEELS_PLUGIN_HH_
#define _MOVE_WHEELS_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int16MultiArray.h"

namespace gazebo
{
  class MoveWheelsPlugin : public ModelPlugin
  {
    // Constactor
    public: MoveWheelsPlugin();

    // Destructor
    public: virtual ~MoveWheelsPlugin();

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

    // ROS subscriber for the joints velocities
    private: ros::Subscriber joints_vels;

    // Callback for the joint velocities
    private: void joints_velocities_callback(const std_msgs::Int16MultiArray::ConstPtr &_msg);

    // offset between left and right side
    private: float left_offset;
    private: float right_offset;
  };
}
#endif
