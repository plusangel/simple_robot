# simple_robot

This is a simple simulation of a differential drive mobile robots for real
experiments. It does not use ros_control package so everything is manual.
You can send joint velocities from your controller framework and get back
encoders ticks counts, like in real robot!

At this configuration it support a 2 two wheel differential drive robot but it
can asily expanded to 4 wheels differential drive robot.

## Configuration

You can find the configuration yaml file inside the simple_robot_gazebo package.
In this file you can set the offset between the left and the right side
(left_offset & right_offset), the standard deviation in the encoders ticks counts
(left_stddev & right_stddev) and the resolution  of the encoders (resolution)

## Instuctions

### Startup
Gazebo simulation:
roslaunch simple_robot_gazebo diff_wheeled_gazebo.launch

Rviz:
roslaunch simple_robot_description view_mobile_robot.launch

Send joint velocities to the robot:
roslaunch simple_robot_control mock_velocities.launch

### Topics

~/encoders (simple_robot_gazebo::encoders msg):
In this topic you can find encoders custom messages published. Those messages contains the
timestamp and the encoder ticks for each joint.

~/odom (nav_msgs::Odometry)
In this topic you can find nav_msgs messages published, indicating the odometry
of the robot based on the gazebo simulation.

~/pose (geometry_msgs::Pose2D)
In this topic you can find Pose2D messages published, indicating the x,y and theta
of the robot based on the gazebo simulation.

~/joint_velocities(std_msgs::Float32MultiArray)
in his topic, we transmit Float32MultiArray messages, which are the velocities for
each joint of the robot.



## Software Setup

Tested in:

Ubuntu Xenial 16.04,

ROS Kinetic,

Gazebo 7.x


## Contributing

### Members
Author:
Angelos Plastropoulos (angelos.plastropoulos@innotecuk.com)

Reviewer:
Artur Gmerek (artur.gmerek@innotecuk.com)
