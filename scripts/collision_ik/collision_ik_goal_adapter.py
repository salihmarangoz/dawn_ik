#! /usr/bin/env python

import rospy
import tf
import numpy as np
from ast import literal_eval
from relaxed_ik_ros1.msg import EEPoseGoals
from relaxed_ik_ros1.msg import JointAngles
from control_msgs.msg import JointTrajectoryControllerState
from dawn_ik.msg import IKGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose, Twist, TransformStamped

current_joint_state = None
traj_cmd_pub = None
dawn_ik_goal_sub = None
init_pos = None
init_rot = None
is_init = False

def quaternion_multiply(q1, q0):
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1
    q =  np.array([-x1*x0 - y1*y0 - z1*z0 + w1*w0,
                    x1*w0 + y1*z0 - z1*y0 + w1*x0,
                   -x1*z0 + y1*w0 + z1*x0 + w1*y0,
                    x1*y0 - y1*x0 + z1*w0 + w1*z0])
    return q

# GET THE GOAL AS DAWN IK GOAL AND PASS TO THE COLLISION IK SOLVER
def dawn_ik_goal_cb(msg):
  if not is_init: return
  if msg.mode != 3:
    rospy.logwarn("Only mode 3 is supported!")
    return
  publish_collision_ik_goal(msg.m1_x, msg.m1_y, msg.m1_z, (msg.m2_w, msg.m2_x, msg.m2_y, msg.m2_z))

# GET THE SOLUTION AS COLLISION IK OUTPUT AND PASS TO THE JOINT TRAJECTORY CONTROLLER
def collision_ik_solution_sub_cb(msg):
  if not is_init: return
  if current_joint_state is None: return
  publish_to_traj_controller(msg.angles.data)

# gets the joint state from the joint trajectory controller, mainly just for reading joint names
def joint_trajectory_state_cb(msg):
  if not is_init: return
  global current_joint_state
  current_joint_state = msg

def publish_collision_ik_goal(x, y, z, quad):
  global now
  if not is_init: return

  p_in = PoseStamped()
  p_in.header.frame_id = world_frame
  p_in.header.stamp = rospy.Time(0)

  p_in.pose.position.x = x 
  p_in.pose.position.y = y 
  p_in.pose.position.z = z 
  p_in.pose.orientation.w = quad[0]
  p_in.pose.orientation.x = quad[1]
  p_in.pose.orientation.y = quad[2]
  p_in.pose.orientation.z = quad[3]
  p_out = tf_listener.transformPose(base_frame, p_in)

  # Correction for collision ik...
  inv_init_trans = -np.array(init_trans)
  corr_position = inv_init_trans + np.array([p_out.pose.position.x, p_out.pose.position.y, p_out.pose.position.z])
  inv_init_rot = ( np.array(init_rot) * np.array([1,-1,-1,-1]) ) / np.linalg.norm(init_rot)
  corr_orientation = quaternion_multiply([p_out.pose.orientation.w, p_out.pose.orientation.x, p_out.pose.orientation.y, p_out.pose.orientation.z], inv_init_rot)
  print(corr_position, corr_orientation)

  goal = EEPoseGoals()
  pose = Pose()
  pose.position.x = corr_position[0]
  pose.position.y = corr_position[1]
  pose.position.z = corr_position[2]
  pose.orientation.w =  corr_orientation[0]
  pose.orientation.x =  corr_orientation[1]
  pose.orientation.y =  corr_orientation[2]
  pose.orientation.z =  corr_orientation[3]
  #pose.orientation = p_out.pose.orientation
  goal.ee_poses.append(pose)
  collision_ik_goal_pub.publish(goal)

# publishes joint positions to the joint trajectory controller
def publish_to_traj_controller(joint_positions, time_from_start=0.1):
  if not is_init: return
  if current_joint_state is None: return
  msg = JointTrajectory()
  msg.joint_names = current_joint_state.joint_names
  if len(current_joint_state.joint_names) != len(joint_positions):
    rospy.logerr("Joint names and joint positions don't align!")
  msg.header.stamp = rospy.Time.now()
  p = JointTrajectoryPoint()
  p.positions = joint_positions
  p.time_from_start = rospy.Duration(time_from_start)
  msg.points.append(p)
  traj_cmd_pub.publish(msg)

if __name__ == "__main__":
  rospy.init_node('follow_path_experiment')

  tf_listener = tf.TransformListener()

  traj_cmd_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=2)
  traj_state_sub = rospy.Subscriber("/head_controller/state", JointTrajectoryControllerState, joint_trajectory_state_cb)
  dawn_ik_goal_sub = rospy.Subscriber("/dawn_ik_solver/ik_goal", IKGoal, dawn_ik_goal_cb)
  collision_ik_solution_sub = rospy.Subscriber("/relaxed_ik/joint_angle_solutions", JointAngles, collision_ik_solution_sub_cb)
  collision_ik_goal_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=2)

  world_frame = rospy.get_param('~world_frame', "world")
  base_frame = rospy.get_param('~base_frame', "head_link_base")
  eef_frame = rospy.get_param('~eef_frame', "head_link_eef")

  # bad idea:
  #tf_listener.waitForTransform(base_frame, eef_frame, rospy.Time(0), rospy.Duration(5.0))
  #(init_pos, init_rot) = tf_listener.lookupTransform(base_frame, eef_frame, rospy.Time(0))

  # $ roslaunch dawn_ik horti_fake.launch
  # $ rosrun relaxed_ik_ros1 rviz_viewer.py
  # copy the values (NOT HERE, INTO THE LAUNCH FILE!)
  init_trans = rospy.get_param('~init_trans', [0., 0., 0.])
  init_trans = literal_eval(init_trans)
  init_rot = rospy.get_param('~init_rot', [1., 0., 0., 0.])
  init_rot = literal_eval(init_rot)

  is_init = True
  rospy.spin()