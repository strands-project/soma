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
    insert_scene = rospy.ServiceProxy('/soma_llsd/insert_scene',InsertScene)
    update_scene = rospy.ServiceProxy('/soma_llsd/update_scene',UpdateScene)
    get_scene = rospy.ServiceProxy('/soma_llsd/get_scene',GetScene)


    # just create a fake blank scene entry
    scene = insert_scene(None,None,None,None,None,None,None,None,None,None)
    if(scene.result is True):

        # update the waypoint field
        print("success!")
        print("scene id: " + scene.response.id)
        print("scene waypoint is blank, look: '" + scene.response.waypoint+"'")
        scene.response.waypoint = "My House"
        update_scene(scene.response)
        print("scene waypoint is now not blank: " + scene.response.waypoint)

    else:
        print("failure!")
