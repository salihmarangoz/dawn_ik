import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_center_joints", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
print("Available groups:", robot.get_group_names())

selected_group_name = None
num_active_joints = 0
for group_name in robot.get_group_names():
    move_group = moveit_commander.MoveGroupCommander(group_name)
    if num_active_joints < len(move_group.get_active_joints()):
        selected_group_name = group_name
        num_active_joints = len(move_group.get_active_joints())

if selected_group_name is None:
    print("No suitable move group found!")
    exit(-1)
else:
    print("Selected move group:", selected_group_name)

move_group = moveit_commander.MoveGroupCommander(selected_group_name)

active_joints = move_group.get_active_joints()
print("Active joints:", active_joints)

joint_goal = []
for j in active_joints:
    min_bound = robot.get_joint(j).min_bound()
    max_bound = robot.get_joint(j).max_bound()
    center_pos = (min_bound+max_bound)/2.0
    joint_goal.append(center_pos)

move_group.go(joint_goal, wait=True)
move_group.stop()