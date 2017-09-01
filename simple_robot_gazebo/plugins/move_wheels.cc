#include "move_wheels.hh"

using namespace gazebo;

MoveWheelsPlugin::MoveWheelsPlugin() : ModelPlugin()
{
}


MoveWheelsPlugin::~MoveWheelsPlugin()
{
}


void MoveWheelsPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  this->model = _parent;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&MoveWheelsPlugin::OnUpdate, this, _1));

  // Safety check
  if (model->GetJointCount() == 0)
  {
    ROS_ERROR("[move_wheels]: Invalid joint count, plugin not loaded");
    return;
  } else
  {
    ROS_INFO("[move_wheels]: Plugin found %d joints on the model", model->GetJointCount());
  }

  jointList = model->GetJoints();

  // Debuging logging
  /*ROS_INFO("The number of links are: %ld", linkList.size());
  */
  for (unsigned i = 0; i < jointList.size(); i++) {
    ROS_INFO("[move_wheels]: (%d): %s", i, (jointList[i]->GetName()).c_str());
  }


  // Load ROS - initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "[move_wheels]: Not loading plugin since ROS hasn't been properly initialized.\n";
    return;
  }

  // ros stuff
  this->rosNode = new ros::NodeHandle("");

  // subscribing to the ros topics, and calling the callback functions
  this->lWheels_vel = this->rosNode->subscribe("/lWheels", 10, &MoveWheelsPlugin::left_velocity_callback, this);
  this->rWheels_vel = this->rosNode->subscribe("/rWheels", 10, &MoveWheelsPlugin::right_velocity_callback, this);


  ROS_INFO("[move_wheels]: MoveWheels Plugin: Loaded succesfully");
}

void MoveWheelsPlugin::left_velocity_callback(const std_msgs::Float32ConstPtr &_msg)
{
  //ROS_INFO("Callback iteration");
  double vl =(double)(_msg->data);
  //ROS_INFO("%f rad/s velocity for left wheels", vl);
  this->jointList[0]->SetParam("fmax", 0, 10000.0);
  this->jointList[0]->SetParam("vel", 0, vl);

}

void MoveWheelsPlugin::right_velocity_callback(const std_msgs::Float32ConstPtr &_msg)
{
  //ROS_INFO("Callback iteration");
  double vr =(double)(_msg->data);
  //ROS_INFO("%f rad/s velocity for right wheels", vr);
  this->jointList[1]->SetParam("fmax", 0, 10000.0);
  this->jointList[1]->SetParam("vel", 0, vr);

}

void MoveWheelsPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  //ROS_INFO("OnUpdate iteration");
  /*
  this->jointList[0]->SetParam("fmax", 0, 10000.0);
  this->jointList[0]->SetParam("vel", 0, 1.0);

  this->jointList[1]->SetParam("fmax", 0, 10000.0);
  this->jointList[1]->SetParam("vel", 0, -1.0);
  */
}

GZ_REGISTER_MODEL_PLUGIN(MoveWheelsPlugin)
