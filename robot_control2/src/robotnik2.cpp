#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float32MultiArray.h"
#include <cstdlib>
#include <math.h>

//these are holding the current control action
float vr1;
float vr2;
float vl1;
float vl2;
float vr;
float vl;


void takeControl(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
 vr1=msg->data[0];
 vr2=msg->data[1];
 vl1=msg->data[2];
 vl2=msg->data[3];
 //where for groundInno vr1=vr2, and vl1=vl2
 vr=vr1;
 vl=vl1;
 ROS_INFO("Received control: [%4.2f %4.2f %4.2f %4.2f]", msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
}

int main(int argc, char **argv)
{
ros::init(argc, argv, "robotnik2");
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("travel_motor_PWM", 1000, takeControl);
ros::Publisher chatter_pub = n.advertise<geometry_msgs::Pose2D>("poza", 1000);//send current position
ros::Rate loop_rate(1);
int count=0;
//these are holding the current localisation:
float x=0;
float y=0;
float theta=0;
float h=0.1;
float W=0.2;
vr=0;
vl=0;
geometry_msgs::Pose2D poza;

while(ros::ok())
{
 //calculate robot's postion
 x=x-h*((vr+vl)/2)*sin(theta);
 y=y+h*((vr+vl)/2)*cos(theta);
 theta=theta+h*(vr-vl)/W;
 //send it:
 poza.x=x;
 poza.y=y;
 poza.theta=theta;

 chatter_pub.publish(poza);
 ROS_INFO("Count: [%d] Loc: [%4.2f, %4.2f,%4.2f]",count, poza.x, poza.y, poza.theta);

 ros::spinOnce();
 loop_rate.sleep();
 ++count;
}

 return 0;
}
//rostopic pub -r 1 /poza geometry_msgs/Pose2D '{x: 2.0, y: 0.0, theta: 3.0}'

