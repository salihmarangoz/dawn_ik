#! /usr/bin/env python

# Author: Salih Marangoz - 2023

# Example usage:
#
# 1. Start your simulation and start recording:
# $ rosbag record /arm_left_controller/follow_joint_trajectory/goal /arm_right_controller/follow_joint_trajectory/goal
#
# 2. Use RViz for moving the robots. It is recommended to plan first then execute movements.
#
# 3. Start this script. This will parse the bag file and publish control_msgs/FollowJointTrajectoryAction messages by
#    removing the time-delay occured while recording.
#
# 4. (Optional) If wait_for_trigger param is set to True, call /moveit_replay_trajectory/trigger service to start the replay.

import rospy
import numpy as np
import sys
import os
import json
import rosbag
import actionlib
import control_msgs.msg
import std_srvs.srv

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
BAG_FILE = SCRIPT_DIR + "/" + "2023-07-19-16-35-58.bag"

def trigger_cb(req):
  global is_triggered
  rospy.loginfo("moveit_replay_trajectory triggered!")
  is_triggered = True
  return True

if __name__ == "__main__":
  rospy.init_node('moveit_replay_trajectory')
  delay = rospy.get_param("~delay", 0.0)
  bag_filename = rospy.get_param("~bag_filename", BAG_FILE)
  num_repeats = rospy.get_param("~num_repeats", 1) # num_repeats<=0 for infinite loop
  wait_for_trigger = rospy.get_param("~wait_for_trigger", False)
  if int(num_repeats) == 0: num_repeats = -1

  is_triggered = not wait_for_trigger
  if wait_for_trigger:
    trigger_service = rospy.Service('~trigger', std_srvs.srv.Empty, trigger_cb)

  # Open bag file
  bag = rosbag.Bag(bag_filename)
  info = bag.get_type_and_topic_info()
  context = {}

  # Find suitable topics
  for topic_name, topic_info in info.topics.items():
      if topic_info.msg_type == "control_msgs/FollowJointTrajectoryActionGoal":
          print("Found topic {} with type control_msgs/FollowJointTrajectoryActionGoal".format(topic_name))
          context[topic_name] = {"messages": []}

  # Read all messages from selected topics
  for topic_name, msg, t in bag.read_messages():
      if topic_name in context:
          context[topic_name]["messages"].append(msg)

  # Check all topics has the same number of values
  n_msgs = -1
  for topic_name, v in context.items():
      if n_msgs < 0:
          n_msgs = len(v["messages"])
      else:
          if len(v["messages"]) != n_msgs:
              print("Failed!")

  # Initialize action clients
  for topic_name, v in context.items():
      parent_topic_name = topic_name.replace("/goal", "")
      v["client"] = actionlib.SimpleActionClient(parent_topic_name, control_msgs.msg.FollowJointTrajectoryAction)
      v["client"].wait_for_server()

  r = rospy.Rate(100.0)
  while not rospy.is_shutdown() and not is_triggered:
    rospy.loginfo_once("moveit_replay_trajectory waiting to be triggered")
    r.sleep()
  if not is_triggered:
    bag.close()
    exit(0)

  # Publish action commands simultaneously
  rospy.loginfo("moveit_replay_trajectory started!")
  while num_repeats != 0 and not rospy.is_shutdown():
    for i in range(n_msgs):
        # Send commands
        rospy.loginfo("Publishing messages. Idx: " + str(i))
        for topic_name, v in context.items():
            goal = v["messages"][i]
            goal.header.stamp = rospy.Time.now()
            v["client"].send_goal(goal.goal)

        # Wait for the server to finish the action
        for topic_name, v in context.items():
            v["client"].wait_for_result()

        if delay > 0:
          rospy.sleep(delay)

    if num_repeats > 0:
      num_repeats -= 1
  rospy.loginfo("moveit_replay_trajectory finished!")

  bag.close()