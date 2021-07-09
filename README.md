# ROS_SCX10_Chassis
Repo containing services scripts and other miscellaneous code for the SCX10 Autonomous Chassis

## Services
Taken from https://blog.roverrobotics.com/how-to-run-ros-on-startup-bootup/ and modified to run in parallel rather than as a single service so that modules can be added and removed later on as needed and restarted without taking down other modules.

Don't forget to chmod +x in /etc/ros && /usr/sbin/ros
