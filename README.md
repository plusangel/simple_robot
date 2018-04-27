# simple_robot

This is a simple simulation of differential drive mobile robots (2 and 4 wheels)
for real experiments. It does not use ros_control package so everything is manual.
You can send joint velocities from your controller framework and get back
encoders ticks counts, like in real robot!

Because of its simplicity you can contuct your experiments with Kalman Filters and
mapping without need to worry about the complexity of your model.

Everything is there simple and visible in your fingertips.


## Configuration

You can find the configuration yaml file inside the simple_robot_gazebo package.
In this file you can set the offset between the left and the right side
(left_offset & right_offset), the standard deviation in the encoders ticks counts
(left_stddev & right_stddev) and the resolution  of the encoders (resolution)


## Execution
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

For robot_pose_ekf case:
```
roslaunch simple_robot_description_2wheels view_mobile_robot_with_gazebo_robot_ekf.launch
```
For robot_localisation case:
```
roslaunch simple_robot_description_2wheels view_mobile_robot_with_gazebo_robot_localization.launch
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


__Manual control: Use keyboard to control the robot (differential drive)__

* For 2wheels model:

```
roslaunch simple_robot_control manual_control_2wheels.launch
```

* For 4wheels model:

```
roslaunch simple_robot_control keyboard_teleop.launch
roslaunch simple_robot_control twist_to_motors_4wheels.launch
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


## IOs

### Topics

~/encoders (simple_robot_gazebo/encoders msg):

In this topic you can find encoders custom messages published. Those messages contains the
timestamp and the encoder ticks for each joint.

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


## License

BSD


## Software Setup

Tested in:

Ubuntu Xenial 16.04,

ROS Kinetic,

Gazebo 7.x


## Misc

[Covariance matrix for /vo and /odom](https://answers.ros.org/question/64759/covariance-matrix-for-vo-and-odom/)


## Contributing

Authors:
Angelos Plastropoulos (angelos.plastropoulos@innotecuk.com)


## Acknowledgements

TBD
