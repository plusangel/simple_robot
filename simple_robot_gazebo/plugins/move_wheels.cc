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
    ROS_ERROR("[motors]: Invalid joint count, plugin not loaded");
    return;
  } else
  {
    ROS_INFO("[motors]: Plugin found %d joints on the model", model->GetJointCount());
  }

  jointList = model->GetJoints();

  // Debuging logging
  /*ROS_INFO("The number of links are: %ld", linkList.size());
  */
  for (unsigned i = 0; i < jointList.size(); i++) {
    ROS_INFO("[motors]: (%d): %s", i, (jointList[i]->GetName()).c_str());
  }


  // Load ROS - initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "[motors]: Not loading plugin since ROS hasn't been properly initialized.\n";
    return;
  }

  // ros stuff
  this->rosNode = new ros::NodeHandle("");
  if (rosNode->hasParam("left_offset") && rosNode->hasParam("right_offset"))
  {
    rosNode->getParam("/left_offset", left_offset);
    rosNode->getParam("/right_offset", right_offset);
    //debug
    ROS_INFO("[motors]: Left offset %f vs Right offset %f", left_offset, right_offset);
  }

  // subscribing to the ros topics, and calling the callback functions
  this->joints_vels = this->rosNode->subscribe("/joint_velocities", 10, &MoveWheelsPlugin::joints_velocities_callback, this);

  ROS_INFO("[motors]: MoveWheels Plugin: Loaded succesfully");
}

void MoveWheelsPlugin::joints_velocities_callback(const std_msgs::Float32MultiArray::ConstPtr &_msg)
{

  //ROS_INFO("Callback iteration");
  auto vels = (_msg->data);
  //ROS_INFO("[motors]: Receive joint velocities %d, %d", vels[0], vels[1]);

  // apply the side offsets
  vels[0] = left_offset*vels[0];
  vels[1] = right_offset*vels[1];

  //ROS_INFO("%f rad/s velocity for left wheels", vl);
  this->jointList[0]->SetParam("fmax", 0, 10000.0);
  this->jointList[0]->SetParam("vel", 0, (double)vels[1]);

  //ROS_INFO("%f rad/s velocity for right wheels", vr);
  this->jointList[1]->SetParam("fmax", 0, 10000.0);
  this->jointList[1]->SetParam("vel", 0, (double)vels[0]);

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
