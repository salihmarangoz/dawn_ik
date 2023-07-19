#! /usr/bin/env python

import rospy
import tf
import numpy as np

from dawn_ik.msg import IKGoal
from moveit_collision_check.srv import CheckCollision

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose, Twist
import sys
from scipy import interpolate
from visualization_msgs.msg import Marker
import time, os
import yaml
import json

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))

AXES = "rxyz"

traj_pub = None
listener = None
current_joint_state = None

def publish_ee_goal(x, y, z, roll, pitch, yaw):
  dawnik_goal = IKGoal()
  dawnik_goal.mode = IKGoal.MODE_1 + IKGoal.MODE_2
  dawnik_goal.m1_x = x
  dawnik_goal.m1_y = y
  dawnik_goal.m1_z = z
  dawnik_goal.m1_limit_dist = 0.1
  dawnik_goal.m1_weight = 4
  quad = tf.transformations.quaternion_from_euler(float(roll), float(pitch), float(yaw), axes=AXES)
  dawnik_goal.m2_x = quad[0]
  dawnik_goal.m2_y = quad[1]
  dawnik_goal.m2_z = quad[2]
  dawnik_goal.m2_w = quad[3]
  dawnik_goal.m2_weight = 10
  dawn_ik_goal_pub.publish(dawnik_goal)

  marker = Marker()
  pose = Pose()
  pose.position.x = x
  pose.position.y = y
  pose.position.z = z
  #quad = tf.transformations.quaternion_from_euler(float(roll), float(pitch), float(yaw), axes=AXES)
  pose.orientation.x =  quad[0]
  pose.orientation.y =  quad[1]
  pose.orientation.z =  quad[2]
  pose.orientation.w =  quad[3]
  marker.header.frame_id = world_frame
  marker.type = marker.SPHERE
  marker.id = 53
  marker.action = marker.ADD
  marker.pose = pose
  marker.lifetime = rospy.Duration()
  marker.scale.x = 0.05
  marker.scale.y = 0.05
  marker.scale.z = 0.05
  marker.color.a = 1.0
  marker.color.r = 1.0
  marker_pub.publish(marker)

def track_joint_state(msg):
  global current_joint_state
  current_joint_state = msg

if __name__ == "__main__":
  rospy.init_node('run_experiment')
  waypoints_file = rospy.get_param("~waypoints_file", SCRIPT_DIR + "/waypoints.txt")
  publish_rate = rospy.get_param("~publish_rate", 100.0)
  world_frame = rospy.get_param("~world_frame", "world")
  endpoint_frame = rospy.get_param("~endpoint_frame", "head_link_eef")
  wait_for_init = rospy.get_param("~wait_for_init", 5.0)
  wait_for_solver = rospy.get_param("~wait_for_solver", 5.0)
  wait_for_shutdown = rospy.get_param("~wait_for_shutdown", 2.0)

  rospy.loginfo("Waiting for the check_collision service...")
  rospy.wait_for_service('/moveit_collision_check/check_collision')
  check_collision = rospy.ServiceProxy('/moveit_collision_check/check_collision', CheckCollision)

  listener = tf.TransformListener()
  dawn_ik_goal_pub = rospy.Publisher("/dawn_ik_solver/ik_goal", IKGoal, queue_size=5)
  marker_pub = rospy.Publisher("~goal_marker", Marker, queue_size = 5)
  rospy.Subscriber("/joint_states", JointState, track_joint_state)

  print("Reading file:", waypoints_file)
  waypoints = np.loadtxt(waypoints_file)
  w_t = waypoints[:,0]
  w_x = waypoints[:,1]
  w_y = waypoints[:,2]
  w_z = waypoints[:,3]
  w_roll = waypoints[:,4]
  w_pitch = waypoints[:,5]
  w_yaw = waypoints[:,6]
  t_max = np.max(w_t)

  f_x = interpolate.interp1d(w_t, w_x)
  f_y = interpolate.interp1d(w_t, w_y)
  f_z = interpolate.interp1d(w_t, w_z)
  f_roll = interpolate.interp1d(w_t, w_roll)
  f_pitch = interpolate.interp1d(w_t, w_pitch)
  f_yaw = interpolate.interp1d(w_t, w_yaw)

  # wait for everything to initialize
  rospy.sleep(wait_for_init)

  # let the solvers to move the arm to the initial state
  t = 0.0
  x = float(f_x(t))
  y = float(f_y(t))
  z = float(f_z(t))
  roll = f_roll(t)
  pitch = float(f_pitch(t))
  yaw = float(f_yaw(t))
  publish_ee_goal(x,y,z,roll,pitch,yaw)

  # wait for the solver to move the robot to the initial pose
  rospy.sleep(wait_for_solver)

  rate = rospy.Rate(publish_rate)
  entries = []
  wait_for_shutdown_remained = wait_for_shutdown
  while not rospy.is_shutdown():
    # Experiment entries...
    entry = {}

    # ENTRY CURRENT TIME
    entry["time"] = t

    # ENTRY: EE GOAL
    x = float(f_x(t))
    y = float(f_y(t))
    z = float(f_z(t))
    roll = f_roll(t)
    pitch = float(f_pitch(t))
    yaw = float(f_yaw(t))
    publish_ee_goal(x,y,z,roll,pitch,yaw)

    # casting to float to avoid an annoying bug which can modifiy the input variables. See the PR: https://github.com/ros/geometry/pull/241
    goal_quad = np.array( tf.transformations.quaternion_from_euler(float(roll), float(pitch), float(yaw), axes=AXES) )
    (goal_roll, goal_pitch, goal_yaw) = tf.transformations.euler_from_quaternion(goal_quad)

    # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    # roll,pitch,yaw are in "rxyz" but goal_roll, goal_pitch, goal_yaw are in "sxyz" which is the default in ROS.
    # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    entry["ee_goal"] = {"x": x, "y": y, "z": z, "roll": goal_roll, "pitch": goal_pitch, "yaw": goal_yaw}

    rate.sleep() #################### wait for robots to move #################################

    # ENTRY: EE CURRENT
    try:
      (curr_trans, curr_rot) = listener.lookupTransform(world_frame, endpoint_frame, rospy.Time(0))
      (curr_roll, curr_pitch, curr_yaw) = tf.transformations.euler_from_quaternion(curr_rot)
      curr_roll = float(curr_roll)
      curr_pitch = float(curr_pitch)
      curr_yaw = float(curr_yaw)
      curr_x = float(curr_trans[0])
      curr_y = float(curr_trans[1])
      curr_z = float(curr_trans[2])
    except Exception as e:
      rospy.logerr("tf error. experiment failed!")
      print(e)
      exit(-1)
    entry["ee_curr"] = {"x": curr_x, "y": curr_y, "z": curr_z, "roll": curr_roll, "pitch": curr_pitch, "yaw": curr_yaw}
    #print(curr_roll, curr_pitch, curr_yaw, "-->", goal_roll, goal_pitch, goal_yaw)

    # ENTRY: theta_diff
    # https://math.stackexchange.com/questions/90081/quaternion-distance
    theta_diff = np.arccos( 2*np.sum((goal_quad*curr_rot))**2 - 1 )
    entry["theta_diff"] = theta_diff
    #print(theta_diff)

    # ENTRY: JOINT STATES
    entry["joint_names"] = list(current_joint_state.name)
    entry["joint_positions"] = list(current_joint_state.position)
    entry["joint_velocities"] = list(current_joint_state.velocity)
    entry["joint_efforts"] = list(current_joint_state.effort)

    entries.append(entry)

    t += 1.0/publish_rate
    if t>t_max:
      t = t_max
      wait_for_shutdown_remained -= 1.0/publish_rate
      if wait_for_shutdown_remained < 0: break

  #################### check for collisions after the experiment is done #################################
  rospy.loginfo("Started offline collision checking")
  total_collisions = 0
  for entry in entries:
    js = JointState()
    js.name = entry["joint_names"]
    js.position = entry["joint_positions"]
    res = check_collision(js) # remove procedure call
    entry["collision_state"] = res.collision_state
    entry["collision_distance"] = res.collision_distance
    total_collisions += int(res.collision_state)
  rospy.loginfo("Finished offline collision checking")
  rospy.loginfo("Total collisions: " + str(total_collisions))
  
  #################### save results to a file #################################
  timestr = time.strftime("%Y%m%d-%H%M%S")
  out_filename_default = SCRIPT_DIR + "/../../results/experiment_" + timestr + ".json" # or yaml
  out_filename = rospy.get_param("~output_file", out_filename_default)
  if out_filename == "": out_filename = out_filename_default

  with open(out_filename, "w") as f:
    #f.write(yaml.dump(entries, default_flow_style=None, sort_keys=False)) # yaml is better at storing human readable data
    f.write(json.dumps(entries)) # json is better at storing machine readable data

  print("=====================================================")
  print("Experiment Finished. Everything will be closed now...")
  print("=====================================================")
