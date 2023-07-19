#!/usr/bin/python

import sys
import math
import rospy
import moveit_commander

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from tf.transformations import quaternion_from_euler

FRAME_ID = 'world'
OBJECT_POSITIONS = {'target_1': [0.4, -0.8, 1.5]}
PICK_ORIENTATION_EULER = [-math.pi, 0, 0]
PLACE_ORIENTATION_EULER = [-math.pi, 0, 0]
SCENE = moveit_commander.PlanningSceneInterface()


def create_collision_object(id, dimensions, pose):
    object = CollisionObject()
    object.id = id
    object.header.frame_id = FRAME_ID

    solid = SolidPrimitive()
    solid.type = solid.BOX
    solid.dimensions = dimensions
    object.primitives = [solid]

    object_pose = Pose()
    object_pose.position.x = pose[0]
    object_pose.position.y = pose[1]
    object_pose.position.z = pose[2]

    object.primitive_poses = [object_pose]
    object.operation = object.ADD
    return object

# http://docs.ros.org/en/jade/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
def add_collision_objects():
    table_1 = create_collision_object(id='table_1',
                                      dimensions=[0.3, 0.6, 0.2],
                                      pose=[0.34, 0.02, 0.1])
    target_1 = create_collision_object(id='target_1',
                                       dimensions=[0.1, 0.1, 0.1],
                                       pose=OBJECT_POSITIONS["target_1"])
    SCENE.add_object(table_1)
    SCENE.add_object(target_1)


def reach_named_position(arm, target):
    arm.set_start_state_to_current_state()
    arm.clear_pose_targets()
    arm.set_named_target(target)
    plan_success, plan, planning_time, error_code = arm.plan()
    return arm.execute(plan, wait=True)

def reach_pose(arm, pose, tolerance=0.01):
    arm.set_start_state_to_current_state()
    arm.clear_pose_targets()
    arm.set_pose_target(pose)
    arm.set_goal_position_tolerance(tolerance)
    return arm.go(wait=True)

def reach_pose_cart(arm, pose, tolerance=0.01):
    arm.set_start_state_to_current_state()
    arm.clear_pose_targets()

    waypoints = []
    #waypoints.append(arm.get_current_pose().pose)
    waypoints.append(pose)
    (plan, fraction) = arm.compute_cartesian_path(
                                 waypoints,   # waypoints to follow
                                 0.01,        # eef_step
                                 0.0)         # jump_threshold
    return arm.execute(plan, wait=True)

def pose(name, arm, x,y,z,roll,pitch,yaw,is_cart=True):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    orientation = quaternion_from_euler(roll, pitch,yaw)
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    if is_cart:
        reach_pose_cart(arm, pose)
    else:
        reach_pose(arm, pose)

def pose1(name, arm):
    pose = Pose()
    pose.position.x = OBJECT_POSITIONS[name][0]
    pose.position.y = OBJECT_POSITIONS[name][1] 
    pose.position.z = OBJECT_POSITIONS[name][2] + 0.051
    orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    reach_pose_cart(arm, pose)

def pose2(name, arm):
    pose = Pose()
    pose.position.x = OBJECT_POSITIONS[name][0]
    pose.position.y = OBJECT_POSITIONS[name][1] * -1
    pose.position.z = OBJECT_POSITIONS[name][2] + 0.051
    orientation = quaternion_from_euler(*PLACE_ORIENTATION_EULER)
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    reach_pose_cart(arm, pose)

def pose3(name, arm):
    pose = Pose()
    pose.position.x = OBJECT_POSITIONS[name][0]
    pose.position.y = 0
    pose.position.z = OBJECT_POSITIONS[name][2] + 0.2
    orientation = quaternion_from_euler(*PLACE_ORIENTATION_EULER)
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    reach_pose_cart(arm, pose)

def pose4(name, arm):
    pose = Pose()
    pose.position.x = OBJECT_POSITIONS[name][0] - 0.25
    pose.position.y = 0
    pose.position.z = OBJECT_POSITIONS[name][2] + 0.15
    orientation = quaternion_from_euler(*PLACE_ORIENTATION_EULER)
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    reach_pose_cart(arm, pose)


def main():
    object_name = "target_1"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('experiment_horti')
    rospy.sleep(2)
    robot = moveit_commander.RobotCommander('robot_description')

    # record and play
    # /arm_left_controller/follow_joint_trajectory/goal
    # /arm_right_controller/follow_joint_trajectory/goal

    scaling_factor = 0.1

    R_X, R_Y, R_Z = 0.349, -1.07, 1.39
    arm_right = moveit_commander.MoveGroupCommander('arm_right', ns=rospy.get_namespace())
    arm_right.set_num_planning_attempts(45)
    arm_right.set_planning_time(10.0)
    arm_right.set_max_acceleration_scaling_factor(scaling_factor)
    arm_right.set_max_velocity_scaling_factor(scaling_factor)

    L_X, L_Y, L_Z = 0.349, -0.366, 1.39
    arm_left = moveit_commander.MoveGroupCommander('arm_left', ns=rospy.get_namespace())
    arm_left.set_num_planning_attempts(45)
    arm_left.set_planning_time(10.0)
    arm_left.set_max_acceleration_scaling_factor(scaling_factor)
    arm_left.set_max_velocity_scaling_factor(scaling_factor)

    H_X, H_Y, H_Z = 0.134, -0.719, 1.7
    arm_head = moveit_commander.MoveGroupCommander('head', ns=rospy.get_namespace())
    arm_head.set_num_planning_attempts(45)
    arm_head.set_planning_time(10.0)
    arm_head.set_max_acceleration_scaling_factor(scaling_factor)
    arm_head.set_max_velocity_scaling_factor(scaling_factor)

    SCENE.remove_world_object("") # clear all objects
    SCENE.remove_attached_object("") # clear all objects
    rospy.sleep(1)


    # init
    arm_head.go([0.0681740124101438, 0.3446005247401807, 0.8620155377363731, -0.0002557286993070757, 0.516957165120407, 0.06839732128340958], wait=True)
    if not rospy.is_shutdown(): pose(object_name, arm_right, R_X+0.05, R_Y, R_Z+0.2, -math.pi/2, 0, 0, is_cart=False)
    if not rospy.is_shutdown(): pose(object_name, arm_left, L_X+0.05, L_Y, L_Z+0.2, math.pi/2, 0, 0, is_cart=False)
    rospy.sleep(5)
    while not rospy.is_shutdown():
        # get closer to the target
        if not rospy.is_shutdown(): pose(object_name, arm_right, R_X+0.05, R_Y+0.3, R_Z+0.2, -math.pi/2, 0, 0, is_cart=False)
        if not rospy.is_shutdown(): pose(object_name, arm_left, L_X+0.05, L_Y-0.3, L_Z+0.2, math.pi/2, 0, 0, is_cart=False)
        rospy.sleep(1)

        # cross-position
        if not rospy.is_shutdown(): pose(object_name, arm_right, R_X+0.05, R_Y+0.3, R_Z+0.2, (-0.8)*math.pi, 0, 0, is_cart=False)
        if not rospy.is_shutdown(): pose(object_name, arm_left, L_X+0.05, L_Y-0.3, L_Z+0.2, (0.8)*math.pi, 0, 0, is_cart=False)
        rospy.sleep(1)

        # lift the fruit
        if not rospy.is_shutdown(): pose(object_name, arm_right, R_X+0.05, R_Y+0.3, R_Z+0.5, (-0.5)*math.pi, 0, 0, is_cart=False)
        if not rospy.is_shutdown(): pose(object_name, arm_left, L_X+0.05, L_Y-0.2, L_Z+0.2, (0.8)*math.pi, 0, 0, is_cart=False)
        rospy.sleep(1)

if __name__ == '__main__':
    main()