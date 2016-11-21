#!/usr/bin/env python

import roslib
import rospy
import sys
import argparse
import os
import uuid
import tf
import std_msgs
from soma_llsd_msgs.msg import Segment,Observation,Scene
from geometry_msgs.msg import Pose,Point,Quaternion
from soma_llsd.srv import *



if __name__ == '__main__':
    insert_scene = rospy.ServiceProxy('/soma_llsd/insert_scene_auto',InsertSceneAuto)

    #string episode_id
    #string waypoint
    #string meta_data
    #uint32 timestamp

    #sensor_msgs/PointCloud2 cloud
    #sensor_msgs/Image rgb_img
    #sensor_msgs/Image depth_img

    #string camera_info_topic
    #string robot_pose_topic

    # just create a fake blank scene entry
    scene = insert_scene(None,"home","{}",61,None,None,None,"/head_xtion/depth/camera_info","/robot_pose")
    if(scene.result is True):

        # update the waypoint field
        print("success!")
        print("scene id: " + scene.response.id)
        print("scene waypoint is now not blank: " + scene.response.waypoint)

    else:
        print("failure!")
