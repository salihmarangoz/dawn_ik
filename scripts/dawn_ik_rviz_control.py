#!/usr/bin/env python

# source: https://github.com/ros-visualization/visualization_tutorials/blob/noetic-devel/interactive_marker_tutorials/scripts/basic_controls.py

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from dawn_ik.msg import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point


class RvizController:

  def __init__(self):
    self.server = InteractiveMarkerServer("dawn_ik_rviz_controls")
    self.goal_pub = rospy.Publisher("/dawn_ik_solver/ik_goal", IKGoal, queue_size=1)
    self.goal = IKGoal()

    self.menu_handler = MenuHandler()
    self.menu_handler.insert( "IDLE", callback=self.processFeedback )
    self.menu_handler.insert( "ENDPOINT_POSE", callback=self.processFeedback )
    self.menu_handler.insert( "ENDPOINT_POSE_ORIENTATION", callback=self.processFeedback )
    self.menu_handler.insert( "ENDPOINT_POSE_LOOK_AT_GOAL", callback=self.processFeedback )
    self.menu_handler.insert( "KEEP_DISTANCE_LOOK_AT_GOAL", callback=self.processFeedback )

    self.header = None
    self.endpoint_pose = None
    self.target_pose = None
    self.menu_entry_id = 1

    position = Point( 0.5, 0, 0)
    self.make6DofMarker("endpoint", InteractiveMarkerControl.MOVE_ROTATE_3D, position, True )
    position = Point( 0.7, 0, 0)
    self.make6DofMarker("target", InteractiveMarkerControl.MOVE_3D, position, True )

    self.server.applyChanges()

  def updateGoal(self):
    IDLE                        = 1
    ENDPOINT_POSE               = 2
    ENDPOINT_POSE_ORIENTATION   = 3
    ENDPOINT_POSE_LOOK_AT_GOAL  = 4
    KEEP_DISTANCE_LOOK_AT_GOAL  = 5

    #############################################################################
    if self.menu_entry_id == IDLE:
      self.goal.mode = 0
    #############################################################################
    elif self.menu_entry_id == ENDPOINT_POSE:
      self.goal.mode = 1
      self.goal.m1_x.value = self.endpoint_pose.position.x
      self.goal.m1_x.weight = 1.0
      self.goal.m1_y.value = self.endpoint_pose.position.y
      self.goal.m1_y.weight = 1.0
      self.goal.m1_z.value = self.endpoint_pose.position.z
      self.goal.m1_z.weight = 1.0
      self.goal.m1_distance.value = 0.0
      self.goal.m1_distance.weight = 1.0
    #############################################################################
    elif self.menu_entry_id == ENDPOINT_POSE_ORIENTATION:
      pass
    #############################################################################
    elif self.menu_entry_id == ENDPOINT_POSE_LOOK_AT_GOAL:
      pass
    #############################################################################
    elif self.menu_entry_id == KEEP_DISTANCE_LOOK_AT_GOAL:
      pass
    #############################################################################



  def processFeedback(self, feedback):
    if feedback.marker_name == "endpoint":
      self.header = feedback.header
      self.endpoint_pose = feedback.pose
    elif feedback.marker_name == "target":
      self.header = feedback.header
      self.target_pose = feedback.pose

    if feedback.menu_entry_id != 0:
      self.header = feedback.header
      self.menu_entry_id = feedback.menu_entry_id

    self.updateGoal()
    self.goal_pub.publish(self.goal)
    self.server.applyChanges()

  def alignMarker(self, feedback):
    pose = feedback.pose
    pose.position.x = round(pose.position.x-0.5)+0.5
    pose.position.y = round(pose.position.y-0.5)+0.5
    self.server.setPose( feedback.marker_name, pose )
    self.server.applyChanges()

  def makeMarker(self, msg):
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.1
    marker.color.g = 0.5
    marker.color.b = 1.0
    marker.color.a = 0.85
    return marker

  def make6DofMarker(self, name, interaction_mode, position, show_6dof = False):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "world"
    int_marker.pose.position = position
    int_marker.scale = 0.3
    int_marker.name = name

    # insert a box
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( self.makeMarker(int_marker) )
    control.interaction_mode = interaction_mode
    int_marker.controls.append( control )

    if show_6dof: 
      control = InteractiveMarkerControl()
      control.orientation.w = 1
      control.orientation.x = 1
      control.orientation.y = 0
      control.orientation.z = 0
      control.name = "rotate_x"
      control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
      int_marker.controls.append(control)

      control = InteractiveMarkerControl()
      control.orientation.w = 1
      control.orientation.x = 1
      control.orientation.y = 0
      control.orientation.z = 0
      control.name = "move_x"
      control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
      int_marker.controls.append(control)

      control = InteractiveMarkerControl()
      control.orientation.w = 1
      control.orientation.x = 0
      control.orientation.y = 1
      control.orientation.z = 0
      control.name = "rotate_z"
      control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
      int_marker.controls.append(control)

      control = InteractiveMarkerControl()
      control.orientation.w = 1
      control.orientation.x = 0
      control.orientation.y = 1
      control.orientation.z = 0
      control.name = "move_z"
      control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
      int_marker.controls.append(control)

      control = InteractiveMarkerControl()
      control.orientation.w = 1
      control.orientation.x = 0
      control.orientation.y = 0
      control.orientation.z = 1
      control.name = "rotate_y"
      control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
      int_marker.controls.append(control)

      control = InteractiveMarkerControl()
      control.orientation.w = 1
      control.orientation.x = 0
      control.orientation.y = 0
      control.orientation.z = 1
      control.name = "move_y"
      control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
      int_marker.controls.append(control)

    self.server.insert(int_marker, self.processFeedback)
    self.menu_handler.apply( self.server, int_marker.name )

if __name__=="__main__":
  rospy.init_node("dawn_ik_rviz_controls_node")
  rc = RvizController()
  rospy.spin()
