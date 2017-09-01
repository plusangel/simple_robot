#include "encoders_ticks.hh"

using namespace gazebo;

EncodersTicksPlugin::EncodersTicksPlugin() : ModelPlugin()
{
}


EncodersTicksPlugin::~EncodersTicksPlugin()
{
}


void EncodersTicksPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  this->model = _parent;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&EncodersTicksPlugin::OnUpdate, this, _1));

  // Safety check
  if (model->GetJointCount() == 0)
  {
    ROS_ERROR("[encoders_ticks]: Invalid joint count, plugin not loaded");
    return;
  } else
  {
    ROS_INFO("[encoders_ticks]: Plugin found %d joints on the model", model->GetJointCount());
  }

  jointList = model->GetJoints();

  // Debuging logging
  /*ROS_INFO("The number of links are: %ld", linkList.size());
  */
  for (unsigned i = 0; i < jointList.size(); i++) {
    ROS_INFO("[encoders_ticks]: (%d): %s", i, (jointList[i]->GetName()).c_str());
  }


  // Load ROS - initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "[encoders_ticks]: Not loading plugin since ROS hasn't been properly initialized.\n";
    return;
  }

  // ros stuff
  this->rosNode = new ros::NodeHandle("");


  ROS_INFO("[encoders_ticks]: EncodersTicks Plugin: Loaded succesfully");
}

void EncodersTicksPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  double time = ros::Time::now().toSec();
  math::Angle left_angle = (this->jointList[0])->GetAngle(0);

  double angle_l = left_angle.Radian();
  ROS_INFO("[encoders_ticks]: Left %f at %f", angle_l*(180/M_PI), time);

  math::Angle right_angle = (this->jointList[1])->GetAngle(0);

  double angle_r = right_angle.Radian();
  ROS_INFO("[encoders_ticks]: Right %f at %f", angle_r*(180/M_PI), time);
}

GZ_REGISTER_MODEL_PLUGIN(EncodersTicksPlugin)
