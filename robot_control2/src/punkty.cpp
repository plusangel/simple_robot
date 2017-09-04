#include "ros/ros.h"
#include "robot_control2/reference.h"
//x,y,th,d
float punkty[5][4]=
{
	{10,15,0,1},
	{0,5,0,1},
	{2,8,0.3,1},
	{3,3,0.3,1},
	{0,10,0,1}
};

bool next_point(robot_control2::reference::Request &req,
	 robot_control2::reference::Response &res)
{
 if(req.a<5){
 res.x=punkty[req.a][0];
 res.y=punkty[req.a][1];
 res.th=punkty[req.a][2];
 res.d=punkty[req.a][3];
 }else
{
 res.x=punkty[4][0];
 res.y=punkty[4][1];
 res.th=punkty[4][2];
 res.d=punkty[4][3];
}

 ROS_INFO("request: a=%d", req.a);
 ROS_INFO("sending back response: [%4.2f %4.2f %4.2f %4.2f]", res.x, res.y, res.th, res.d);
 return true;
}

int main(int argc, char **argv)
{
ros::init(argc, argv, "punkty");
ros::NodeHandle n;

ros::ServiceServer service = n.advertiseService("punkty_adve",next_point);
ROS_INFO("Ready to send points.");
ros::spin();

return 0;
}
