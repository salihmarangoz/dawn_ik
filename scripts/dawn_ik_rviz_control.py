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
    self.goal_pub = rospy.Publisher("dawn_ik_solver/ik_goal", IKGoal, queue_size=1)
    self.goal = IKGoal()

    self.menu_handler = MenuHandler()
    self.menu_handler.insert( "IDLE", callback=self.processFeedback )
    self.menu_handler.insert( "ENDPOINT_POSE", callback=self.processFeedback )
    self.menu_handler.insert( "ENDPOINT_POSE_ORIENTATION", callback=self.processFeedback )
    self.menu_handler.insert( "ENDPOINT_POSE_ORIENTATION (Adaptive)", callback=self.processFeedback )
    self.menu_handler.insert( "ENDPOINT_POSE + LOOK-AT-GOAL", callback=self.processFeedback )

    self.header = None
    self.endpoint_pose = None
    self.target_pose = None
    self.menu_entry_id = 1
    self.feedback = None

    position = Point( 0.5, 0, 0)
    self.make6DofMarker("endpoint", InteractiveMarkerControl.MOVE_ROTATE_3D, position, True )
    position = Point( 0.7, 0, 0)
    #self.make6DofMarker("target", InteractiveMarkerControl.MOVE_3D, position, True )

    self.server.applyChanges()

  def updateGoal(self):
    self.goal = IKGoal()

    IDLE                        = 1
    ENDPOINT_POSE               = 2
    ENDPOINT_POSE_ORIENTATION   = 3
    ENDPOINT_POSE_ORIENTATION_A = 4
    ENDPOINT_POSE_LOOKATGOAL    = 5

    #############################################################################
    if self.menu_entry_id == IDLE:
      self.goal.mode = IKGoal.MODE_0
    #############################################################################
    elif self.menu_entry_id == ENDPOINT_POSE:
      self.goal.mode = IKGoal.MODE_1
      self.goal.m1_x = self.endpoint_pose.position.x
      self.goal.m1_y = self.endpoint_pose.position.y
      self.goal.m1_z = self.endpoint_pose.position.z
      self.goal.m1_limit_dist = 0.1
      self.goal.m1_weight = 4.0
    #############################################################################
    elif self.menu_entry_id == ENDPOINT_POSE_ORIENTATION:
      self.goal.mode = IKGoal.MODE_1 + IKGoal.MODE_2
      self.goal.m1_x = self.endpoint_pose.position.x
      self.goal.m1_y = self.endpoint_pose.position.y
      self.goal.m1_z = self.endpoint_pose.position.z
      self.goal.m1_limit_dist = 0.1
      self.goal.m1_weight = 4.0
      self.goal.m2_w = self.endpoint_pose.orientation.w
      self.goal.m2_x = self.endpoint_pose.orientation.x
      self.goal.m2_y = self.endpoint_pose.orientation.y
      self.goal.m2_z = self.endpoint_pose.orientation.z
      self.goal.m2_weight = 1.0
    #############################################################################
    elif self.menu_entry_id == ENDPOINT_POSE_ORIENTATION_A:
      self.goal.mode = IKGoal.MODE_1 + IKGoal.MODE_2
      self.goal.m1_x = self.endpoint_pose.position.x
      self.goal.m1_y = self.endpoint_pose.position.y
      self.goal.m1_z = self.endpoint_pose.position.z
      self.goal.m1_limit_dist = 0.1
      self.goal.m1_weight = 4.0
      self.goal.m2_w = self.endpoint_pose.orientation.w
      self.goal.m2_x = self.endpoint_pose.orientation.x
      self.goal.m2_y = self.endpoint_pose.orientation.y
      self.goal.m2_z = self.endpoint_pose.orientation.z
      self.goal.m2_weight = 1.0
      if "move" in self.feedback.control_name:
        self.goal.m2_weight = 5.0
      if "rotate" in self.feedback.control_name:
        self.goal.m1_weight = 5.0
    #############################################################################
    elif self.menu_entry_id == ENDPOINT_POSE_LOOKATGOAL:
      self.goal.mode = IKGoal.MODE_1 + IKGoal.MODE_3
      self.goal.m1_x = self.endpoint_pose.position.x
      self.goal.m1_y = self.endpoint_pose.position.y
      self.goal.m1_z = self.endpoint_pose.position.z
      self.goal.m1_weight = 1.0
      self.goal.m3_x = self.target_pose.position.x
      self.goal.m3_y = self.target_pose.position.y
      self.goal.m3_z = self.target_pose.position.z
      self.goal.m3_weight = 1.0


  def processFeedback(self, feedback):
    self.feedback = feedback
    if feedback.marker_name == "endpoint":
      self.header = feedback.header
      # qnorm = (feedback.pose.orientation.w**2 + feedback.pose.orientation.x**2 + feedback.pose.orientation.y**2 + feedback.pose.orientation.z**2)**0.5
      # feedback.pose.orientation.w /= qnorm
      # feedback.pose.orientation.x /= qnorm
      # feedback.pose.orientation.y /= qnorm
      # feedback.pose.orientation.z /= qnorm
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
    control.name = "move_3d"
    control.always_visible = True
    control.markers.append( self.makeMarker(int_marker) )
    control.interaction_mode = interaction_mode
    int_marker.controls.append( control )

    if show_6dof: 
      control = InteractiveMarkerControl()
      control.orientation.w = 2**-0.5
      control.orientation.x = 2**-0.5
      control.orientation.y = 0
      control.orientation.z = 0
      control.name = "rotate_x"
      control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
      int_marker.controls.append(control)

      control = InteractiveMarkerControl()
      control.orientation.w = 2**-0.5
      control.orientation.x = 2**-0.5
      control.orientation.y = 0
      control.orientation.z = 0
      control.name = "move_x"
      control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
      int_marker.controls.append(control)

      control = InteractiveMarkerControl()
      control.orientation.w = 2**-0.5
      control.orientation.x = 0
      control.orientation.y = 2**-0.5
      control.orientation.z = 0
      control.name = "rotate_z"
      control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
      int_marker.controls.append(control)

      control = InteractiveMarkerControl()
      control.orientation.w = 2**-0.5
      control.orientation.x = 0
      control.orientation.y = 2**-0.5
      control.orientation.z = 0
      control.name = "move_z"
      control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
      int_marker.controls.append(control)

      control = InteractiveMarkerControl()
      control.orientation.w = 2**-0.5
      control.orientation.x = 0
      control.orientation.y = 0
      control.orientation.z = 2**-0.5
      control.name = "rotate_y"
      control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
      int_marker.controls.append(control)

      control = InteractiveMarkerControl()
      control.orientation.w = 2**-0.5
      control.orientation.x = 0
      control.orientation.y = 0
      control.orientation.z = 2**-0.5
      control.name = "move_y"
      control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
      int_marker.controls.append(control)

    self.server.insert(int_marker, self.processFeedback)
    self.menu_handler.apply( self.server, int_marker.name )

if __name__=="__main__":
  rospy.init_node("dawn_ik_rviz_controls_node")
  rc = RvizController()
  rospy.spin()
