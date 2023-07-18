#! /usr/bin/env python
'''
author: Salih Marangoz
original code is based on https://github.com/uwgraphics/relaxed_ik_ros1/blob/629d2d81904cb5f405718ba3842299cf21aeaa9a/src/rviz_viewer.py
'''

import numpy as np
import rospkg
import rospy
import tf
import os

import yaml

from geometry_msgs.msg import Point, PoseStamped
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

# LOCAL IMPORTS
import transformations as T
from export_to_collision_ik_obstacles import collision_obj_to_name

def is_point(pt):
    if len(pt) < 3:
        return False
    for e in pt:
        try:
            float(e)
        except ValueError:
            return False
    return True

path_to_src = rospkg.RosPack().get_path('relaxed_ik_ros1')
animation_folder_path = path_to_src + '/animation_files/'
geometry_folder_path = path_to_src + '/geometry_files/'
env_settings_file_path = path_to_src + '/relaxed_ik_core/config/settings.yaml'

def print_cb(msg):
    p = msg.pose.position
    print(msg.marker_name + " is now at [" + str(p.x) + ", " + str(p.y) + ", " + str(p.z) + "]")

def make_marker(name, fixed_frame, shape, scale, ts, quat, is_dynamic, 
                points=None, color=[0.0,0.0,1.0,0.75], marker_scale=0.3):                
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "head_" + fixed_frame # TODO: added a prefix for visualization
    int_marker.name = name
    int_marker.pose.position.x = ts[0]
    int_marker.pose.position.y = ts[1]
    int_marker.pose.position.z = ts[2]
    int_marker.pose.orientation.x = quat[1]
    int_marker.pose.orientation.y = quat[2]
    int_marker.pose.orientation.z = quat[3]
    int_marker.pose.orientation.w = quat[0]
    int_marker.scale = marker_scale

    if not shape == 'widget':
        marker = Marker()
        marker.scale.x = scale[0] * 2
        marker.scale.y = scale[1] * 2
        marker.scale.z = scale[2] * 2
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        if shape == "cuboid":
            marker.type = Marker.CUBE
        elif shape == "sphere":
            marker.type = Marker.SPHERE
        elif shape == "point_cloud":
            marker.type = Marker.POINTS
            marker.points = points

        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        int_marker.controls.append(control)

    return int_marker

def set_collision_world(server, fixed_frame, env_settings):
    dyn_obs_handles = []

    if 'obstacles' in env_settings:
        obstacles = env_settings['obstacles']
    else:
        raise NameError('Please define the obstacles in the environment!')

    if 'cuboids' in obstacles: 
        cuboids = obstacles['cuboids']
        if cuboids is not None:
            for c in cuboids:
                if c['animation'] == 'static':
                    is_dynamic = 0
                elif c['animation'] == 'interactive':
                    is_dynamic = 1
                
                c_quat = T.quaternion_from_euler(c['rotation'][0], c['rotation'][1], c['rotation'][2])
                int_marker = make_marker(c['name'], fixed_frame, "cuboid", c['scale'], 
                                        c['translation'], c_quat, is_dynamic)
                server.insert(int_marker, print_cb)

    if 'spheres' in obstacles:
        spheres = obstacles['spheres']
        if spheres is not None:
            for s in spheres:
                if s['animation'] == 'static':
                    is_dynamic = 0
                elif s['animation'] == 'interactive':
                    is_dynamic = 1
                int_marker = make_marker(s['name'], fixed_frame, "sphere", [s['scale']] * 3, 
                                        s['translation'], [1.0,0.0,0.0,0.0], is_dynamic)
                server.insert(int_marker, print_cb)

    if 'point_cloud' in obstacles: 
        point_cloud = obstacles['point_cloud']
        if point_cloud is not None:
            for pc in point_cloud:
                pc_path = geometry_folder_path + pc['file']
                pc_scale = pc['scale']
                pc_points = []
                with open(pc_path, 'r') as point_cloud_file:
                    lines = point_cloud_file.read().split('\n')
                    for line in lines:
                        pt = line.split(' ')
                        if is_point(pt):
                            point = Point()
                            point.x = float(pt[0]) * pc_scale[0]
                            point.y = float(pt[1]) * pc_scale[1]
                            point.z = float(pt[2]) * pc_scale[2]
                            pc_points.append(point)
                
                if pc['animation'] == 'static':
                    is_dynamic = 0
                elif pc['animation'] == 'interactive':
                    is_dynamic = 1
                pc_quat = T.quaternion_from_euler(pc['rotation'][0], pc['rotation'][1], pc['rotation'][2])
                int_marker = make_marker(pc['name'], fixed_frame, "point_cloud", [0.01, 0.01, 0.01], pc['translation'], pc_quat, is_dynamic, points=pc_points)
                server.insert(int_marker, print_cb)

    server.applyChanges()
    return dyn_obs_handles

def main():
    rospy.init_node('rviz_viewer')
    tf_prefix = rospy.get_param('~tf_prefix', "")
    world_frame = rospy.get_param('~world_frame', "world")
    base_frame = rospy.get_param('~base_frame', "head_link_base")
    tf_listener = tf.TransformListener()

    # Parse info from Collision IK
    env_settings_file = open(env_settings_file_path, 'r')
    env_settings = yaml.load(env_settings_file, Loader=yaml.FullLoader)
    if 'loaded_robot' in env_settings:
        info_file_name = env_settings['loaded_robot']['name']
    else:
        raise NameError("Please defined the loaded robot!")
    info_file_path = path_to_src + '/relaxed_ik_core/config/info_files/' + info_file_name
    info_file = open(info_file_path, 'r')
    y = yaml.load(info_file, Loader=yaml.FullLoader)
    fixed_frame = tf_prefix + y['fixed_frame']
    rospy.sleep(0.5)
    server = InteractiveMarkerServer("simple_marker")
    set_collision_world(server, fixed_frame, env_settings)


    # Parse cfg from Dawn IK
    dyn_obs_info = {}
    SCRIPT_PATH = os.path.dirname(os.path.abspath(__file__))
    YAML_PATH = SCRIPT_PATH + "/../../cfg/horti.yaml"
    with open(YAML_PATH, "r") as stream:
        try:
            dawn_ik_cfg = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            exit(-1)
    collision_ik_cfg = {"obstacles":{ "spheres": [] }}
    for obj in dawn_ik_cfg["proximity"]["objects"]:
        obj_name = collision_obj_to_name(obj)
        dyn_obs_info[obj_name] = {"translation": obj["translation"], "link": obj["link"]}

    rate = rospy.Rate(30) # same rate as dawn_ik
    while not rospy.is_shutdown():
        for name, obj in dyn_obs_info.items():
            p_in = PoseStamped()
            p_in.header.frame_id = obj["link"] # may be the robot's base frame?
            p_in.header.stamp = rospy.Time(0)
            p_in.pose.position.x = obj["translation"][0]
            p_in.pose.position.y = obj["translation"][1]
            p_in.pose.position.z = obj["translation"][2]
            p_in.pose.orientation.w = 1.0
            tf_listener.waitForTransform(base_frame, obj["link"], p_in.header.stamp, rospy.Duration(5.0))
            p_out = tf_listener.transformPose(base_frame, p_in)
            server.setPose(name, p_out.pose)

        server.applyChanges()
        rate.sleep()

if __name__ == '__main__':
    main()
