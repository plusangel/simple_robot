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
    ROS_ERROR("[encoders]: Invalid joint count, plugin not loaded");
    return;
  } else
  {
    ROS_INFO("[encoders]: Plugin found %d joints on the model", model->GetJointCount());
  }

  jointList = model->GetJoints();

  // Debuging logging
  /*ROS_INFO("The number of links are: %ld", linkList.size());
  */
  for (unsigned i = 0; i < jointList.size(); i++) {
    ROS_INFO("[encoders]: (%d): %s", i, (jointList[i]->GetName()).c_str());
  }


  // Load ROS - initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "[encoders]: Not loading plugin since ROS hasn't been properly initialized.\n";
    return;
  }

  // ros stuff
  this->rosNode = new ros::NodeHandle("");

  if (rosNode->hasParam("left_stddev") && rosNode->hasParam("right_stddev") && rosNode->hasParam("resolution"))
  {
    rosNode->getParam("/left_stddev", left_stddev);
    rosNode->getParam("/right_stddev", right_stddev);
    rosNode->getParam("/resolution", encoders_resolution);

    //debug
    ROS_INFO("[encoders]: Left stddev %f vs Right stddev %f with resolution %d", left_stddev, right_stddev, encoders_resolution);
  }

  encoders_pub = rosNode->advertise<simple_robot_gazebo::encoders>("encoders", 1);

  ROS_INFO("[encoders]: EncodersTicks Plugin: Loaded succesfully");
}

void EncodersTicksPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  float encoders_multiplier = encoders_resolution/360;
  // get the ROS timeStamp
  double time = ros::Time::now().toSec();
  time *= pow(10, 3);
  uint milliTime = (uint) time;

  // get the left joint angle
  math::Angle left_angle = (this->jointList[1])->GetAngle(0);

  double angle_left = left_angle.Radian();
  double encoder_left = angle_left*(180/M_PI);
  encoder_left = encoders_multiplier * encoder_left;

  // apply a normal distribution to the ticks count
  std::normal_distribution<double> distributionLeft(encoder_left, left_stddev);
  encoder_left = distributionLeft(generator);

  // debug
  //ROS_INFO("[encoders]: Left %f at %d", encoder_left, milliTime);

  // get the right joint angle
  math::Angle right_angle = (this->jointList[0])->GetAngle(0);

  double angle_right = right_angle.Radian();
  double encoder_right = angle_right*(180/M_PI);

  // apply a normal distribution to the ticks count
  std::normal_distribution<double> distributionRight(encoder_right, right_stddev);
  encoder_right = distributionRight(generator);

  // debug
  //ROS_INFO("[encoders]: Right %f at %d", encoder_right, milliTime);

  // publish
  simple_robot_gazebo::encoders msg;
  msg.encoderTicks = {(float)encoder_left, (float)encoder_right};
  msg.timeStamp = milliTime;
  encoders_pub.publish(msg);

}

GZ_REGISTER_MODEL_PLUGIN(EncodersTicksPlugin)
