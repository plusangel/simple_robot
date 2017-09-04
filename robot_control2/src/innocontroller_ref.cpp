#include "ros/ros.h"
#include "robot_control2/reference.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float32MultiArray.h"
#include <cstdlib>
#include <math.h>
#include <iostream>

geometry_msgs::Pose2D curPose; //this variable is holding the current localisation of a robot
geometry_msgs::Pose2D refPos; //reference position

//control parameters
const float eps1=1; //position deadzone
const float eps2=0.5; //rotataion deadzone
const float K1=2; //position gain
const float K2=0.6; //rotation gain

robot_control2::reference srv;

void poseCall(const geometry_msgs::Pose2D::ConstPtr& msg)
{
 curPose.x=msg->x;
 curPose.y=msg->y;
 curPose.theta=msg->theta;
 ROS_INFO("robot loc: [%4.2f %4.2f %4.2f]", msg->x, msg->y, msg->theta);
}

int main(int argc, char **argv)
{
int punkt=0;//point number

ros::init(argc, argv, "innocontroller_ref");
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("pose", 10, poseCall);
//send velocities to Arduino's communication node
ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("joint_velocities", 10);
ros::Rate loop_rate(10);
int count=0;

ros::ServiceClient client=n.serviceClient<robot_control2::reference>("punkty_adve");

while(ros::ok())
{

if(punkt==0){
refPos.x=0;
refPos.y=0;
refPos.theta=0;
}else{
refPos.x=srv.response.x;
refPos.y=srv.response.y;
refPos.theta=srv.response.th;
}

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
float th1=atan2(dy,dx);//-1.570796;//-1*atan2(dx,dy);

ROS_INFO("theta1: %4.2f",th1);

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
	  //stop and request another point
          srv.request.a = punkt++;
 	  if(client.call(srv))
          {
           ROS_INFO("New point: [%4.2f %4.2f %4.2f %4.2f]", srv.response.x, srv.response.y, srv.response.th,  srv.response.d);
          }
          else
          {
           ROS_ERROR("Failed to call service punkty_adve");
           return 1;
          }
 	 d1=0;
	}
	 ROS_INFO("d1: %4.2f",d1);
	if((dx<0) && (dy<0)) //for future modifications:
	{
 	 C1=K1*d1;
	}else{
 	 C1=K1*d1;
	}
}

ROS_INFO("C1: %4.2f",C1);
ROS_INFO("C2: %4.2f",C2);
vRR=C1-C2;
vLL=C1+C2;

//pack everything into std_msgs/Float32MultiArray and send
std_msgs::Float32MultiArray Pos;
Pos.layout.dim.push_back(std_msgs::MultiArrayDimension());
Pos.layout.dim[0].label="poss";
Pos.layout.dim[0].size=4;
Pos.layout.dim[0].stride=1;

Pos.data.clear();
Pos.data.push_back(vRR);
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
