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
  public:

    // Constactor
    MoveWheelsPlugin();

    // Destructor
    virtual ~MoveWheelsPlugin();

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

    // ROS subscriber for the joints velocities
    ros::Subscriber joints_vels;

    // Callback for the joint velocities
    void joints_velocities_callback(const std_msgs::Int16MultiArray::ConstPtr &_msg);

    // offset between left and right side
    float left_offset;
    float right_offset;
  };
}
#endif
