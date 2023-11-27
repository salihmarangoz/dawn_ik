#! /usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from dawn_ik.msg import IKGoal
import numpy as np
import tf

ik_goal = None
def callback(data):
  global ik_goal
  ik_goal = data

rospy.init_node('rviz_marker')
listener = tf.TransformListener()
marker_pub = rospy.Publisher("/look_at_objective_marker", Marker, queue_size = 2)
goal_sub = rospy.Subscriber("/dawn_ik_solver/ik_goal", IKGoal, callback)

marker_current = Marker()
marker_target = Marker()
marker_modified = Marker()

# ee
marker_current.header.frame_id = "link_eef"
marker_current.header.stamp = rospy.Time.now()
marker_current.type = 2 # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
marker_current.id = 0
marker_current.scale.x = 0.05
marker_current.scale.y = 0.05
marker_current.scale.z = 0.05
marker_current.color.r = 0.0
marker_current.color.g = 1.0
marker_current.color.b = 0.0
marker_current.color.a = 1.0
marker_current.pose.position.x = 0
marker_current.pose.position.y = 0
marker_current.pose.position.z = 0
marker_current.pose.orientation.x = 0.0
marker_current.pose.orientation.y = 0.0
marker_current.pose.orientation.z = 0.0
marker_current.pose.orientation.w = 1.0
marker_current.frame_locked = True

# target
marker_target.header.frame_id = "world"
marker_target.header.stamp = rospy.Time.now()
marker_target.type = 2 # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
marker_target.id = 1
marker_target.scale.x = 0.05
marker_target.scale.y = 0.05
marker_target.scale.z = 0.05
marker_target.color.r = 0.0
marker_target.color.g = 0.0
marker_target.color.b = 1.0
marker_target.color.a = 1.0
marker_target.pose.position.x = 0
marker_target.pose.position.y = 0
marker_target.pose.position.z = 0
marker_target.pose.orientation.x = 0.0
marker_target.pose.orientation.y = 0.0
marker_target.pose.orientation.z = 0.0
marker_target.pose.orientation.w = 1.0

while not rospy.is_shutdown():
  marker_pub.publish(marker_current)

  if ik_goal is not None:
    if ik_goal.m3_x != 0.0:
      marker_target.pose.position.x = ik_goal.m3_x
      marker_target.pose.position.y = ik_goal.m3_y
      marker_target.pose.position.z = ik_goal.m3_z
      marker_pub.publish(marker_target)




  rospy.sleep(0.1)