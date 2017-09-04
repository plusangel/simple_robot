#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float32MultiArray.h"
#include <cstdlib>
#include <math.h>
#include <iostream>

geometry_msgs::Pose2D curPose;//this variable is holding the current localisation of a robot

//control parameters
const float eps1=7; //position deadzone
const float eps2=0.05; //rotataion deadzone
const float K1=0.2; //position gain
const float K2=0.1; //rotation gain

void poseCall(const geometry_msgs::Pose2D::ConstPtr& msg)
{
 curPose.x=msg->x;
 curPose.y=msg->y;
 curPose.theta=msg->theta;
 ROS_INFO("robot loc: [%4.2f %4.2f %4.2f]", msg->x, msg->y, msg->theta);
}

int main(int argc, char **argv)
{
//reference position:
geometry_msgs::Pose2D refPos;
refPos.x=50;
refPos.y=50;
refPos.theta=0;

ros::init(argc, argv, "innocontroller2");
//connect to Angelos' localisation node
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("poza", 1000, poseCall);
//send velocities to Arduino's communication node
ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("travel_motor_PWM", 1000);
ros::Rate loop_rate(1);
int count=0;
 
while(ros::ok())
{
//calculate controller's action
//Controller No 1 (combined rotation and position action)
int stan=0;//current state of the algorithm
//0 - initial state of the algorithm
//1 - rotation to be on the line
//2 - rotation to get required theta
float C1=0;
float C2=0;
float vRR=0;
float vLL=0;

float dx=refPos.x-curPose.x;
float dy=refPos.y-curPose.y;
float th1=-1*atan(refPos.x/refPos.y);
float dth=th1-curPose.theta;
float d2=dth;
//float d1=0;

if(std::abs(d2)<=eps2)
{
 d2=0;
}

if(std::abs(d2)>eps2)//correct info
{
 C2=K2*d2;
 ROS_INFO("d2: %4.2f",d2);
}else
{
	float d1=sqrt(pow(dx,2)+pow(dy,2));//error metrics
	if(std::abs(d1)<=eps1)
	{
 	 d1=0;
	}
	 ROS_INFO("d1: %4.2f",d1);
	if((dx<0) && (dy<0))
	{
 	 C1=-K1*d1;
	}else{
 	 C1=K1*d1;
	}
}

ROS_INFO("C1: %4.2f",C1);
ROS_INFO("C2: %4.2f",C2);
vRR=C1+C2;
vLL=C1-C2;

//pack everything into std_msgs/Float32MultiArray and send
std_msgs::Float32MultiArray Pos;
Pos.layout.dim.push_back(std_msgs::MultiArrayDimension());
Pos.layout.dim[0].label="poss";
Pos.layout.dim[0].size=4;
Pos.layout.dim[0].stride=1;

Pos.data.clear();
Pos.data.push_back(vRR);
Pos.data.push_back(vRR);
Pos.data.push_back(vLL);
Pos.data.push_back(vLL);

chatter_pub.publish(Pos);
ROS_INFO("Count: [%d] Control variables: [%4.2f, %4.2f]",count,vRR,vLL);

 ros::spinOnce();
 loop_rate.sleep();
 ++count;
}
 return 0;
}
//rostopic pub -r 1 /poza geometry_msgs/Pose2D '{x: 2.0, y: 0.0, theta: 3.0}'

