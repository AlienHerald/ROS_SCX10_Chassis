# ROS_SCX10_Chassis
Repo containing services scripts and other miscellaneous code for the SCX10 Autonomous Chassis

## Services
Taken from https://blog.roverrobotics.com/how-to-run-ros-on-startup-bootup/ and modified to run in parallel rather than as a single service so that modules can be added and removed later on as needed and restarted without taking down other modules.

Don't forget to chmod +x in /etc/ros && /usr/sbin/ros

## Twist
I need to update with the link to the original tutorial that explains the ROS node. I heavily added to it to control an ESC and servos using a standard PCA9685 using Adafruit Circuitpython libraries.
