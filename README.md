# simple_robot stack

This is a simple simulation of differential drive mobile robots (2 and 4 wheels)
for real experiments. It does not use ros_control package so everything is manual.
You can send joint velocities from your controller framework and get back
encoders ticks counts, like in real robot!

Using this package you can easily find and edit:
- encoders (resolution, noise)
- motors (imperfect amplifiers or noise)
- IMUs
- odometry covariances
- ground truth pose of your robot

Because of its simplicity you can contuct your experiments with Kalman Filters
(robot_pose_ekf and robot_localization) and mapping without need to worry about
the complexity of your model.

Everything is here simple and visible in your fingertips.


## Install directly from gitlab

Open terminal and navigate to src folder in your catkin workspace:
```
cd ~/catkin_ws/src
```

Then clone the repository using:
```
git clone https://gitlab.com/innotecuk-public/simple_robot.git
```

Once it is completed, do the following:
```
cd ~/catkin_ws && catkin_make
```

## Configuration

You can find the configuration yaml file inside the simple_robot_gazebo package.
In this file you can set the offset between the left and the right side
(left_offset & right_offset), the standard deviation in the encoders ticks counts
(left_stddev & right_stddev) and the resolution  of the encoders (resolution)


## Execution

__2 wheels robot simulation (all included)__
```
roslaunch simple_robot robot.launch

```

__Gazebo simulation__

* For 2wheels model (you must set in config/motors_parameters.yaml num_of_wheels: 2):

```
roslaunch simple_robot_gazebo robot_2wheels.launch
```

* For 4wheels model (you must set in config/motors_parameters.yaml num_of_wheels: 4):

```
roslaunch simple_robot_gazebo robot_4wheels.launch
```


__Rviz__

* For 2wheels model (with gazebo simulation run in parallel or not):

```
roslaunch simple_robot_description_2wheels view_mobile_robot.launch
```

* For robot_pose_ekf case:
```
roslaunch simple_robot_description_2wheels view_mobile_robot_with_gazebo_robot_ekf.launch
```

* For robot_localisation case:
```
roslaunch simple_robot_description_2wheels view_mobile_robot_with_gazebo_robot_localization.launch
```

* For 4wheels model:

```
roslaunch simple_robot_description_4wheels view_mobile_robot.launch
```


__Manual control: Use keyboard to control the robot (differential drive)__

* For 2wheels model:

```
roslaunch simple_robot_control manual_control_2wheels.launch
```

* For 4wheels model:

```
roslaunch simple_robot_control manual_control_4wheels.launch
```


__Sensor Fusion using Extended Kalman Filters (please change to compatible IMU plugin in your gazebo file)__

* robot_pose_ekf (extended Kalman Filter): Use extended Kalman filter to fuse encoders and IMU:
```
roslaunch simple_robot_control robot_pose_ekf.launch
```

* robot_localization (extended Kalman Filter): Use extended Kalman filter to fuse encoders and IMU:
```
roslaunch simple_robot_control robot_localisation.launch
```

__Testing__
Send joint velocities to the robot

* For 2wheels model:

```
roslaunch simple_robot_test test_hardware_interface_2wheels.launch
```

* For 4wheels model:

```
roslaunch simple_robot_test test_hardware_interface_4wheels.launch
```

Send cmdvel messages to the robot

* For 2wheels model:

```
roslaunch simple_robot_test test_cmdvel_2wheels.launch
```


## IOs
~/encoders (simple_robot_gazebo/encoders msg)

In this topic you can find encoders custom messages published. Those messages contains the
timestamp and the encoder ticks for each joint.

~/cmd_vel (geometry_msgs/Twist)

The standard topic to accept velocity (linear and angular) commands.

~/imu_data (sensor_msgs/Imu)

The standard imu message. 

~/odom (nav_msgs/Odometry)

In this topic you can find nav_msgs messages published, indicating the odometry
of the robot based on the encoders' data.

~/joint_velocities (std_msgs/Float32MultiArray)

In his topic, we transmit Float32MultiArray messages, which are the velocities for
each joint of the robot.

~/ground_truth/state (nav_msgs/Odometry)

This is the ground truth odometry taken directly from the gazebo simulation.

~/ground_truth/pose (geometry_msgs/PoseStamped)

This is the ground truth robot pose taken directly from the gazebo simulation.

~/robot_pose_ekf/odom_combined (geometry_msgs/PoseWithCovarianceStamped)

The output of the Extended Kalman Filter using robot_pose_ekf

~/odometry/filtered (nav_msgs/Odometry)

The output of the Extended Kalman Filter using robot_localization


## Comments
In order to use the hector IMU (suitable for robot localisation), you need to install the [hector_gazebo_plugins](http://wiki.ros.org/hector_gazebo_plugins)


## Todo
[update odometry for skid steer drive](http://docs.ros.org/jade/api/gazebo_plugins/html/gazebo__ros__skid__steer__drive_8cpp_source.html)


## License
MIT


## Software Setup
Tested in:

Ubuntu Xenial 16.04,

ROS Kinetic,

Gazebo 7.x


## Misc
[Covariance matrix for /vo and /odom](https://answers.ros.org/question/64759/covariance-matrix-for-vo-and-odom/)


## Contributing
[Angelos Plastropoulos](angelos.plastropoulos@innotecuk.com)
