#! /usr/bin/env python

import rospy
import tf
import numpy as np

from dawn_ik.msg import IKGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose, Twist
import sys
from scipy import interpolate
from visualization_msgs.msg import Marker

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
  dawnik_goal.m1_weight = 1
  quad = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
  dawnik_goal.m2_x = quad[0]
  dawnik_goal.m2_y = quad[1]
  dawnik_goal.m2_z = quad[2]
  dawnik_goal.m2_w = quad[3]
  dawnik_goal.m2_weight = 1
  dawn_ik_goal_pub.publish(dawnik_goal)

  marker = Marker()
  pose = Pose()
  pose.position.x = x
  pose.position.y = y
  pose.position.z = z
  #quad = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
  pose.orientation.x =  quad[0]
  pose.orientation.y =  quad[1]
  pose.orientation.z =  quad[2]
  pose.orientation.w =  quad[3]
  rangedik_goal.ee_poses.append(pose)
  marker.header.frame_id = "world"
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
  rospy.init_node('publish_waypoints')
  waypoints_file = rospy.get_param("~waypoints_file", "waypoints.txt")
  publish_rate = rospy.get_param("~publish_rate", 100.0)

  listener = tf.TransformListener()
  dawn_ik_goal_pub = rospy.Publisher("dawn_ik_solver/ik_goal", IKGoal, queue_size=5)
  marker_pub = rospy.Publisher("goal_marker", Marker, queue_size = 5)
  rospy.Subscriber("joint_state", JointState, track_joint_state)

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

  rate = rospy.Rate(publish_rate)
  t = np.min(w_t)
  loop_counter = 0
  waited_beforehand = False
  lines = []
  while not rospy.is_shutdown():
    t += 1/publish_rate
    if t>0 and t>=t_max:
      t = t % t_max
      loop_counter += 1
      break

    if t>=0 and not waited_beforehand:
      waited_beforehand = True
      rospy.sleep(1.0)

    publish_ee_goal(f_x(t),f_y(t),f_z(t),f_roll(t),f_pitch(t),f_yaw(t))

    rate.sleep()

    if loop_counter==0 and t>=0:
      try:
        (test_trans, test_rot) = listener.lookupTransform('/world', '/head_link_eef', rospy.Time(0))
        gt_rot = tf.transformations.quaternion_from_euler(f_roll(t),f_pitch(t),f_yaw(t))
        gt_trans = [f_x(t),f_y(t),f_z(t)]
        lines.append([t, test_trans[0], test_trans[1], test_trans[2], test_rot[0], test_rot[1], test_rot[2], test_rot[3],
                      gt_trans[0], gt_trans[1], gt_trans[2], gt_rot[0], gt_rot[1], gt_rot[2], gt_rot[3]])
      except:
        print("tf error")

  #rospy.spin()

  # save results to a file
  import time, os
  timestr = time.strftime("%Y%m%d-%H%M%S")
  script_dir = os.path.dirname(os.path.realpath(__file__))
  out_filename_default = script_dir+"/../results/experiment_"+timestr+".csv"
  out_filename = rospy.get_param("~out_filename", out_filename_default)
  if out_filename == "":
    out_filename = out_filename_default
  np.savetxt(out_filename, np.asarray(lines), delimiter=",")