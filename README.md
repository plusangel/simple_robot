# simple_robot

This is a simple simulation of a differential drive mobile robots for real 
experiments. It does not use ros_control package so everything is manual. 
You can send joint velocities from your controller framework and get back 
encoders ticks counts, like in real robot!

## Configuration

You can find the configuration yaml file inside the simple_robot_gazebo package. 
In this file you can set the offset between the left and the right side 
(left_offset & right_offset), the standard deviation in the encoders ticks counts 
(left_stddev & right_stddev) and the resolution  of the encoders (resolution)

## Instuctions

Gazebo simulation:
roslaunch simple_robot_gazebo diff_wheeled_gazebo.launch

Rviz:

Check the model: roslaunch innoclimber_visual view_model.launch


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
