#include "ros/ros.h"
#include "simple_robot_control/reference.h"

//This matrix contains points in a format (x, y, th, d)
float points[5][4]=
{
	{10,15,0,1},
	{0,5,0,1},
	{2,8,0.3,1},
	{3,3,0.3,1},
	{0,10,0,100}
};

bool next_point(simple_robot_control::reference::Request &req, simple_robot_control::reference::Response &res)
{
	if(req.a<5){
		res.x=points[req.a][0];
		res.y=points[req.a][1];
		res.th=points[req.a][2];
		res.d=points[req.a][3];
	} else {
		res.x=points[4][0];
		res.y=points[4][1];
		res.th=points[4][2];
		res.d=points[4][3];
	}

	ROS_INFO("[point_generator]: request: a=%ld", req.a);
	ROS_INFO("[point_generator]: sending back response: [%4.2f %4.2f %4.2f %4.2f]", res.x, res.y, res.th, res.d);
	return true;
}

int main(int argc, char **argv)
{
ros::init(argc, argv, "points");
ros::NodeHandle n;

ros::ServiceServer service = n.advertiseService("points_generator",next_point);
ROS_INFO("[point_generator]: ready to send points.");
ros::spin();

return 0;
}
