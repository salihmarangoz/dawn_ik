#!/usr/bin/env python  
import roslib
import rospy
from dawn_ik.msg import IKGoal
import tf

def callback(msg):
  # https://docs.ros.org/en/jade/api/tf/html/python/tf_python.html#tf.TransformBroadcaster.sendTransform
  br.sendTransform((msg.m1_x, msg.m1_y, msg.m1_z),
                   (msg.m2_x, msg.m2_y, msg.m2_z, msg.m2_w),
                   rospy.Time.now(),
                   "goal",
                   "world")

if __name__ == '__main__':
  rospy.init_node('publish_tf_goal_pose')
  br = tf.TransformBroadcaster()
  goal_sub = rospy.Subscriber("/dawn_ik_solver/ik_goal", IKGoal, callback)
  rospy.spin()