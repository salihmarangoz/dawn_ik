#!/usr/bin/env python

# Kills the Gazebo Simulator which rejects dying in the hands of programmer, eliminating waiting time while shutting down gzclient and gzserver

# 1. Copy this file into scripts folder in your simulator package (which includes your models and worlds)
# 2. Make it executable (chmod +x gazebo_killer_node.py)
# 3. Insert the line below into your launch file which starts gazebo and loads your world
# <node name="gazebo_killer_node" pkg="YOUR_PACKAGE_NAME" type="gazebo_killer_node.py"/>

import rospy
import os

rospy.init_node("gazebo_killer")

# wait for shutdown signal
rospy.logwarn("Gazebo killer is ready!")
rospy.spin()

# shutdown gzserver and gzclient
rospy.logwarn("Killing gzserver and gzclient")
for i in range(5):
    os.system("killall -9 gzserver")
    os.system("killall -9 gzclient")