#ifndef _MOVE_WHEELS_PLUGIN_HH_
#define _MOVE_WHEELS_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Int32.h"

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

    // ROS topic callback
    public: void left_velocity_callback(const std_msgs::Int32ConstPtr &_msg);

    // ROS topic callback
    public: void right_velocity_callback(const std_msgs::Int32ConstPtr &_msg);



    // Pointer to the model
    private: physics::ModelPtr model;

    // List which holds the innoclimber joints
    private: physics::Joint_V jointList;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;



    // ROS NodeHanle
    private: ros::NodeHandle* rosNode;

    // ROS subscriber for the left wheels velocities
    private: ros::Subscriber lWheels_vel;

    // ROS subscriber for the right wheels velocities
    private: ros::Subscriber rWheels_vel;

  };
}
#endif
