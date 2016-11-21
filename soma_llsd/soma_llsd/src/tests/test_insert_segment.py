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
    insert_segment = rospy.ServiceProxy('/soma_llsd/insert_segment',InsertSegment)
    update_segment = rospy.ServiceProxy('/soma_llsd/add_observations_to_segment',AddObservationsToSegment)
    get_segment = rospy.ServiceProxy('/soma_llsd/get_segment',GetSegment)

    update_scene = rospy.ServiceProxy('/soma_llsd/update_scene',UpdateScene)
    insert_scene = rospy.ServiceProxy('/soma_llsd/insert_scene',InsertScene)
    # first create a fake blank scene to attach this segment to
    scene = insert_scene("ep_id_01","My Kitchen","{meta_data:'None'}",1475575619,None,None,None,None,None,None)

    if(scene.result is True):
        print("added scene")
    else:
        print("couldn't add scene")
        sys.exit()

    # no meta data, linked to the scene above, with no observations initially
    segment = insert_segment("{meta_data:'None'}",scene.response.id,[])

    if(segment.result is True):
        print("added segment")
    else:
        print("couldn't add segment")
        sys.exit()

    print("trying to update scene")
    scene.response.waypoint = "A new waypoint"
    update_scene(scene.response)
    print("updated scene")
