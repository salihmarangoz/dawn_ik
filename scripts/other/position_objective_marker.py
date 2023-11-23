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
marker_pub = rospy.Publisher("/position_objective_marker", Marker, queue_size = 2)
goal_sub = rospy.Subscriber("/dawn_ik_solver/ik_goal", IKGoal, callback)

marker_current = Marker()
marker_target = Marker()
marker_modified = Marker()

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

marker_modified.header.frame_id = "world"
marker_modified.header.stamp = rospy.Time.now()
marker_modified.type = 2 # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
marker_modified.id = 2
marker_modified.scale.x = 0.05
marker_modified.scale.y = 0.05
marker_modified.scale.z = 0.05
marker_modified.color.r = 1.0
marker_modified.color.g = 0.0
marker_modified.color.b = 0.0
marker_modified.color.a = 1.0
marker_modified.pose.position.x = 0
marker_modified.pose.position.y = 0
marker_modified.pose.position.z = 0
marker_modified.pose.orientation.x = 0.0
marker_modified.pose.orientation.y = 0.0
marker_modified.pose.orientation.z = 0.0
marker_modified.pose.orientation.w = 1.0


while not rospy.is_shutdown():
  marker_pub.publish(marker_current)

  if ik_goal is not None:
    marker_target.pose.position.x = ik_goal.m1_x
    marker_target.pose.position.y = ik_goal.m1_y
    marker_target.pose.position.z = ik_goal.m1_z
    marker_pub.publish(marker_target)

    # according to the paper notes

    try:
      (trans,rot) = listener.lookupTransform('world', 'link_eef', rospy.Time(0))

      targ_pos = np.array([ik_goal.m1_x, ik_goal.m1_y, ik_goal.m1_z])
      curr_pos = np.array([trans[0], trans[1], trans[2]])
      goal_targ_dist = np.linalg.norm(targ_pos-curr_pos)
      scaling = min(ik_goal.m1_limit_dist, goal_targ_dist) / goal_targ_dist
      modified_pos = scaling * (targ_pos-curr_pos) + curr_pos

      marker_modified.pose.position.x = modified_pos[0]
      marker_modified.pose.position.y = modified_pos[1]
      marker_modified.pose.position.z = modified_pos[2]
      marker_pub.publish(marker_modified)
      
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue





  rospy.sleep(0.1)