# ROS2 Leader Follower Epucks

The objective of this package is to simulate a pair of mobile robots, in which a follower robot
follows the leader. The simulation is based on Webots robot simulator which is integrated
with ROS2. Each robot has its own controller. The positions of the robots in 3D space is
published on /odom topic simultaneously. Five different ways points as shown on the Fig.1
are defined. Based on the information received from /odom, the leader robot estimates its
heading direction and translational velocity to reach the waypoints. In parallel, the follower robot
is following the leader.

<img src="https://drive.google.com/uc?export=view&id=1uZLY60qn5xydXm3K9_rnUXL-gU9VeL7f " width="500">

__Figure 1: Leader‐follower robot navigation control with E‐puck robots in Webots__

## Kinematic Model

<img src="https://drive.google.com/uc?export=view&id=1DDdyuT6rFKGuAoqtoiZGuz5LeyBH7yK9 " width="500">

The control signals are designed to drive the robot from its actual configuration to the goal
position:

<img src="https://drive.google.com/uc?export=view&id=1D_YGDXm_TbE3iabWBW7-hTVwf_kRIEWK " width="180">

whereas: 

<img src="https://drive.google.com/uc?export=view&id=1CUttzwNiOKkDnN7H0CGnwzM31m0gkXZZ " width="300">

## Video Demonstration:
[![Alt text](https://img.youtube.com/vi/H3x42sRlvdc/0.jpg)](https://www.youtube.com/watch?v=H3x42sRlvdc)




For more information about ROS2, webots, and epuck, please visit the following links: 

* [Example: E-puck](https://github.com/cyberbotics/webots_ros2/wiki/Example-E-puck)
* [Tutorial: E-puck for ROS2 Beginners](https://github.com/cyberbotics/webots_ros2/wiki/Tutorial-E-puck-for-ROS2-Beginners)
* [ROS2 Driver for Physical E-puck](https://github.com/cyberbotics/epuck_ros2)
