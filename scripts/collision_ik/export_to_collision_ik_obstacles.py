#! /usr/bin/env python

import yaml
import os
from ast import literal_eval
import rospy
import tf
from geometry_msgs.msg import PoseStamped

def collision_obj_to_name(obj):
    obj_name = obj["link"] + obj["shape"] + str(obj["translation"])
    obj_name = obj_name.replace(".", "_")
    obj_name = obj_name.replace(" ", "-")
    obj_name = obj_name.replace(",", "=")
    obj_name = obj_name.replace("(", "")
    obj_name = obj_name.replace(")", "")
    obj_name = obj_name.replace("[", "")
    obj_name = obj_name.replace("]", "")
    obj_name = "".join(obj_name)
    return obj_name

def get_text_inside_parentheses(text):
    start = text.find('(')
    end = text.find(')')

    if start != -1 and end != -1:
        return text[start + 1:end]
    else:
        return None

def get_text_left_of_parentheses(text):
    start = text.find('(')

    if start != -1:
        return text[:start].strip()
    else:
        return text.strip()

def parse_obj_parameters(obj):
    shape_name = get_text_left_of_parentheses(obj["shape"])
    shape_param = get_text_inside_parentheses(obj["shape"])
    return shape_name, shape_param

##########################################################################################



if __name__ == "__main__":
    SCRIPT_PATH = os.path.dirname(os.path.abspath(__file__))
    YAML_PATH = SCRIPT_PATH + "/../../cfg/horti.yaml"
    WORLD_FRAME = "head_link_base"

    rospy.init_node('export_to_collision_ik_obstacles')
    tf_listener = tf.TransformListener()

    with open(YAML_PATH, "r") as stream:
        try:
            dawn_ik_cfg = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            exit(-1)

    collision_ik_cfg = {"obstacles":{ "spheres": [] }}
    for obj in dawn_ik_cfg["proximity"]["objects"]:

        if "head" in obj["link"]: continue # TODO: skip the head arm

        obj_name = collision_obj_to_name(obj)
        obj_shape_name, obj_shape_param = parse_obj_parameters(obj)


        in_pos = obj["translation"]
        p_in = PoseStamped()
        p_in.header.frame_id = obj["link"] # may be the robot's base frame?
        p_in.header.stamp = rospy.Time(0)
        p_in.pose.position.x = in_pos[0]
        p_in.pose.position.y = in_pos[1]
        p_in.pose.position.z = in_pos[2]
        p_in.pose.orientation.w = 1.0
        tf_listener.waitForTransform(WORLD_FRAME, obj["link"], p_in.header.stamp, rospy.Duration(5.0))
        p_out = tf_listener.transformPose(WORLD_FRAME, p_in)
        transformed_pose = [float(p_out.pose.position.x), float(p_out.pose.position.y), float(p_out.pose.position.z)]

        # TODO: assuming that shapes are spheres
        ci_obj = {"name": obj_name, "scale": float(obj_shape_param), "translation": transformed_pose, "animation": "interactive"}
        collision_ik_cfg["obstacles"]["spheres"].append(ci_obj)

    print(yaml.dump(collision_ik_cfg, default_flow_style=None, sort_keys=False))