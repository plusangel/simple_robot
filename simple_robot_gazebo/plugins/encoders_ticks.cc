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
  encoders_pub = rosNode->advertise<simple_robot_gazebo::encoders>("encoders", 1);

  ROS_INFO("[encoders_ticks]: EncodersTicks Plugin: Loaded succesfully");
}

void EncodersTicksPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  double time = ros::Time::now().toSec();
  time *= pow(10, 3);
  uint milliTime = (uint) time;
  math::Angle left_angle = (this->jointList[0])->GetAngle(0);

  double angle_left = left_angle.Radian();
  double encoder_left = angle_left*(180/M_PI);
  ROS_INFO("[encoders_ticks]: Left %f at %d", encoder_left, milliTime);

  math::Angle right_angle = (this->jointList[1])->GetAngle(0);

  double angle_right = right_angle.Radian();
  double encoder_right = angle_right*(180/M_PI);
  ROS_INFO("[encoders_ticks]: Right %f at %d", encoder_right, milliTime);

  simple_robot_gazebo::encoders msg;
  msg.encoderTicks = {(float)encoder_left, (float)encoder_right};
  msg.timeStamp = milliTime;
  encoders_pub.publish(msg);

}

GZ_REGISTER_MODEL_PLUGIN(EncodersTicksPlugin)
