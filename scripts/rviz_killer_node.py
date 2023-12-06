#!/usr/bin/env python

# 1. Copy this file into scripts folder in your simulator package (which includes your models and worlds)
# 2. Make it executable (chmod +x rviz_killer_node.py)
# 3. Insert the line below into your launch file which starts RViz
# <node name="rviz_killer_node" pkg="YOUR_PACKAGE_NAME" type="rviz_killer_node.py"/>

import rospy
import os

rospy.init_node("rviz_killer")

# wait for shutdown signal
rospy.logwarn("RViz killer is ready!")
rospy.spin()
rospy.logwarn("Killing rviz")
for i in range(5):
    os.system("killall -9 rviz")
    os.system("killall -9 rviz")