#include "ros/ros.h"
#include "simple_robot_control/reference.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float32MultiArray.h"
#include <cstdlib>
#include <math.h>
#include <iostream>

geometry_msgs::Pose2D curPose; //this variable is holding the current localisation of a robot
geometry_msgs::Pose2D refPos; //reference position

simple_robot_control::reference srv;

void poseCall(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  curPose.x=msg->x;
  curPose.y=msg->y;
  curPose.theta=msg->theta;
  //ROS_INFO("[controller]: robot location: [%4.2f %4.2f %4.2f]", msg->x, msg->y, msg->theta);
}

int main(int argc, char **argv)
{
  //control parameters
  float eps1=0.5; //position deadzone
  float eps2=0.4; //rotataion deadzone
  float eps3=0.05;
  float K1=0.6; //position gain
  float K2=1.5; //rotation gain

  float granica_rot=eps2;

  int point=0;//point number
  int num_of_wheels = std::stoi(argv[1]);

  ROS_INFO(">>>> %d", std::stoi(argv[1]));

  ros::init(argc, argv, "controller");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("pose", 10, poseCall);
  //send velocities to Arduino's communication node
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("joint_velocities", 10);
  ros::Rate loop_rate(10);
  int count=0;

  ros::ServiceClient client=n.serviceClient<simple_robot_control::reference>("points_generator");

  ROS_INFO("[controller]: Waiting for service");
  client.waitForExistence(ros::Duration(5.0));

  while(ros::ok())
  {

    if(point==0){
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

    //ROS_INFO("theta1: %4.2f",th1);

    float dth=th1-curPose.theta;
    float d2=dth;
    //float d1=0;

    if(std::abs(d2)<=eps3)
    {
      d2=0;
    }

    if(std::abs(d2)>granica_rot)//correct info
    {
      C2=K2*d2;
      ROS_INFO("[controller]: d2 %4.2f",d2);
      granica_rot=eps3;
    }else
    {
      granica_rot=eps2;
      float d1=sqrt(pow(dx,2)+pow(dy,2));//error metrics
      if(std::abs(d1)<=eps1)
      {
        //stop and request another point
        srv.request.a = point++;
        if(client.call(srv))
        {
          ROS_INFO("[controller]: new point [%4.2f %4.2f %4.2f %4.2f]", srv.response.x, srv.response.y, srv.response.th,  srv.response.d);
        }
        else
        {
          ROS_ERROR("[controller]: Failed to call service waypoints_generator");
          return 1;
        }
        d1=0;
      }
      ROS_INFO("[controller]: d1 %4.2f",d1);
      if((dx<0) && (dy<0)) //for future modifications:
      {
        C1=K1*d1;
      }else{
        C1=K1*d1;
      }
    }

    ROS_INFO("[controller]: C1 %4.2f",C1);
    ROS_INFO("[controller]: C2 %4.2f",C2);
    vRR=C1-C2;
    vLL=C1+C2;

    //pack everything into std_msgs/Float32MultiArray and send
    std_msgs::Float32MultiArray Pos;
    Pos.layout.dim.push_back(std_msgs::MultiArrayDimension());
    Pos.layout.dim[0].label="poss";
    Pos.layout.dim[0].size=1;
    Pos.layout.dim[0].stride=num_of_wheels;

    Pos.data.clear();

    if (num_of_wheels == 2) {
      Pos.data.push_back(vRR);
      Pos.data.push_back(vLL);
    } else if (num_of_wheels == 4) {
      Pos.data.push_back(vLL);
      Pos.data.push_back(vRR);
      Pos.data.push_back(vLL);
      Pos.data.push_back(vRR);
    }

    chatter_pub.publish(Pos);
    ROS_INFO("[controller]: Control variables: [%4.2f, %4.2f, %4.2f, %4.2f]", vLL, vRR, vLL, vRR);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
    if(srv.response.d==100){
      return 0;
    }
  }
  return 0;
}
