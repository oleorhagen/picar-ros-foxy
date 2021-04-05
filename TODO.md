* DONE Get an X-window running with ROS2
  CLOSED: [2021-03-20 Sat 17:27]


  See [here](http://wiki.ros.org/docker/Tutorials/GUI) for instructions

  For a nice VNC example:
  https://automaticaddison.com/how-to-install-and-launch-ros2-using-docker/


* INPROGRESS Skim through all the ROS2 tutorials

  [here](https://docs.ros.org/en/foxy/Tutorials.html)

  Got here: https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html

* INPROGRESS  Get the simple URDF description from my old project running in ROS2

  * Read up on Rviz2 - and it's description format
  * Read up on Gazebo - and it's description format (if it has changed)...

* TODO Create a simple TF2 transform to move the robot in the world frame in Rviz

  Also check the type of message that is /odom

  Also, how do ROS2 controllers publish to /tf to move the robot ?

* DONE - Write a simple keyboard teleop node

  Not needed - Use teleop_twist_keyboard

  teleop_twist_joy can be used with PS controllers

* INPROGRESS - Write a simply controller in Python, as a first step to ROS2 control integration

  Later on migrate to ROS2 control, but first simply have a node which:

  listens to:
  /cmd_vel

  publishes:
  /cmd_vel_out
  /odom

* TODO - Write an Ackermann controller for ROS2 control

  Start
  [here](https://ros-controls.github.io/control.ros.org/getting_started.html),
  and check out the accompanying GitHub repo in order to have a template to start from

  * https://github.com/ros-controls/ros2_control
  * https://github.com/ros-controls/ros2_controllers

  NOTE: The ROS1 Ackermann controller is a multi-interface controller,
  containing both a velocity controller, and a postition controller for the
  speed, and wheel turn respectively.

  The
  [diff-drive-controller](https://ros-controls.github.io/control.ros.org/getting_started.html)
  is probably a good starting point.

** INPROGRESS Hardware description in URDF (Motor[actuator]) and (Position[actuator]).
** First step - simply write a velocity controller, to get the thing to interface with ROS2, and move the actual robot, then later create the multi-controller interface
** Add Odometry later, by integrating the wheels lateral movements.
** How to add hardware to the controller?
** Have it running in Gazebo

* TODO Have a teleop node working with the rc-car

Can probably start with a standard keyboard setup (exists), and then move to the playstation setup.

* TODO Follow all the ROS2 navigation tutorials to get the simulation moving again

https://navigation.ros.org/

* TODO Add nodes for interfacing with the actual PiCar

This https://github.com/GigaFlopsis/rc_car_ros is a nice example of what I want in ROS1

* Misc

  ROS package index:
  https://index.ros.org/r/teleop_tools/github-ros-teleop-teleop_tools/

  https://github.com/GigaFlopsis/rc_car_ros Nice project (ROS1)

  This has joy->ackermann transformations
  https://github.com/sngc1/ros2_rc_car

  Interesting tutorials
  https://industrial-training-master.readthedocs.io/en/melodic/_source/session7/ROS2-Basics.html
