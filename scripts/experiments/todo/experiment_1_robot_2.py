#!/usr/bin/python

import sys
import math
import rospy
import moveit_commander

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from tf.transformations import quaternion_from_euler

FRAME_ID = 'robot2/world'
OBJECT_POSITIONS = {'target_1': [0.35, -0.21, 0.26]}
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
    rospy.init_node('experiment_1')
    rospy.sleep(2)

    arm = moveit_commander.MoveGroupCommander('xarm7', ns=rospy.get_namespace())
    robot = moveit_commander.RobotCommander('robot_description')
    gripper = robot.get_joint('joint7')

    arm.set_num_planning_attempts(45)
    arm.set_planning_time(10.0)
    arm.set_max_acceleration_scaling_factor(1.0)
    arm.set_max_velocity_scaling_factor(1.0)

    SCENE.remove_world_object("") # clear all objects
    SCENE.remove_attached_object("") # clear all objects
    arm.go([0, 0, 0, 1.8505337665018249, 0, 0.7143234149940315, 0], wait=True) # xarm7
    add_collision_objects()
    rospy.sleep(1)
    #reach_named_position(arm=arm, target='home')

    while not rospy.is_shutdown():
        if not rospy.is_shutdown():
            pose3(object_name, arm=arm)
            rospy.sleep(1)
        if not rospy.is_shutdown():
            pose1(object_name, arm=arm)
            arm.attach_object(object_name)
            rospy.sleep(1)
        if not rospy.is_shutdown():
            pose3(object_name, arm=arm)
            rospy.sleep(1)
        if not rospy.is_shutdown():
            pose4(object_name, arm=arm)
            rospy.sleep(1)
        if not rospy.is_shutdown():
            pose3(object_name, arm=arm)
            rospy.sleep(1)
        if not rospy.is_shutdown():
            pose2(object_name, arm=arm)
            arm.detach_object(object_name)
            rospy.sleep(1)
        if not rospy.is_shutdown():
            pose3(object_name, arm=arm)
            rospy.sleep(1)
        if not rospy.is_shutdown():
            pose2(object_name, arm=arm)
            arm.attach_object(object_name)
            rospy.sleep(1)
        if not rospy.is_shutdown():
            pose3(object_name, arm=arm)
            rospy.sleep(1)
        if not rospy.is_shutdown():
            pose1(object_name, arm=arm)
            arm.detach_object(object_name)
            rospy.sleep(1)

if __name__ == '__main__':
    main()