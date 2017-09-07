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

### Starting up the simulation
__Gazebo simulation__
* For 2wheels model:

```
roslaunch simple_robot_gazebo robot_2wheels.launch

```
* For 4wheels model:

```
roslaunch simple_robot_gazebo robot_4wheels.launch
```



__Rviz__
* For 2wheels model:

```
roslaunch simple_robot_description_2wheels view_mobile_robot.launch
```

* For 4wheels model:

```
roslaunch simple_robot_description_4wheels view_mobile_robot.launch
```


__Manual control: Send joint velocities to the robot__
* For 2wheels model:

```
roslaunch simple_robot_control test_velocities_2wheels.launch
```

* For 4wheels model:

```
roslaunch simple_robot_control test_velocities_4wheels.launch
```

__Manual control: Use keyboard to control the robot__
* For 2wheels model:
 
```
roslaunch simple_robot_control keyboard_teleop.launch
roslaunch simple_robot_control twist_to_motors_2wheels.launch
```

* For 4wheels model:
 
```
roslaunch simple_robot_control keyboard_teleop.launch
roslaunch simple_robot_control twist_to_motors_4wheels.launch
```


__Autonomous control: Use the position controller to move the robot__
```
roslaunch simple_robot_control controller.launch
```

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
Authors:
Angelos Plastropoulos (angelos.plastropoulos@innotecuk.com)

Artur Gmerek (artur.gmerek@innotecuk.com)
