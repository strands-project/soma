#!/usr/bin/env python

import roslib; roslib.load_manifest("soma_roi_manager")
import rospy
from rospkg import RosPack
import json
import argparse
import random
import copy
import sys

from threading import Timer

from mongodb_store.message_store import MessageStoreProxy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import PoseArray

from soma_roi_manager.srv import *

from soma_msgs.msg import SOMAROIObject


def trapezoidal_shaped_func(a, b, c, d, x):
    min_val = min(min((x - a)/(b - a), float(1.0)), (d - x)/(d - c))
    return max(min_val, float(0.0))


def r_func(x):
    a = -0.125
    b =  0.125
    c =  0.375
    d =  0.625

    x = 1.0 - x

    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value

def g_func(x):
    a =  0.125
    b =  0.375
    c =  0.625
    d =  0.875

    x = 1.0 - x

    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value


def b_func(x):
    a =  0.375
    b =  0.625
    c =  0.875
    d =  1.125

    x = 1.0 - x

    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value


class SOMAROIDrawer():

    def __init__(self):

        # default file
        rp = RosPack()

        ''' Set the default color as blue '''
        self.rgb = [0.0,0.0,1.0]
        #self.rgb[0] = 0.0
        #self.rgb[1] = 0.0
        #self.rgb[2] = 1.0

        self._msg_store=MessageStoreProxy(database="somadata",collection="roi")

        s = rospy.Service('soma/draw_roi', DrawROI, self.handle_draw_roi)

       # Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
        self.markerpub = rospy.Publisher("soma_roi_marker_array", MarkerArray, queue_size=1)

        rospy.spin()

    def handle_draw_roi(self,req):
        if(len(req.rgb)==3):
            self.rgb = req.rgb
        #if not req.draw_all:
        #    return DrawROIResponse(self.load_objects(req.map_name,req.roi_id,req.roi_config,False))
        #elif req.draw_all:
        return DrawROIResponse(self.load_objects(req.map_name,req.roi_id,req.roi_config,req.draw_all,req.draw_mostrecent))

        #return True


    def _delete_markers(self):
        marker = Marker()
        marker.action = 3
        marker.header.frame_id = "map"
        markerarray = MarkerArray()
        markerarray.markers.append(marker)
        self.markerpub.publish(markerarray)

    def coords_to_lnglat(x, y):
        earth_radius = 6371000.0 # in meters
        lng = 90 - math.degrees(math.acos(float(x) / earth_radius))
        lat = 90 - math.degrees(math.acos(float(y) / earth_radius))
        return [lng , lat]

    def _retrieve_objects(self, map_name, roi_id, roi_config, draw_all,draw_mostrecent):
        if draw_all:
            objs = self._msg_store.query(SOMAROIObject._type, message_query={"map_name": map_name,
                                                                      })
            return objs
        if roi_config == "":
            if not draw_mostrecent:
                objs = self._msg_store.query(SOMAROIObject._type, message_query={"map_name": map_name,
                                                                      "id": roi_id})
            else:
                objs = self._msg_store.query(SOMAROIObject._type, message_query={"map_name": map_name,
                                                                      "id": roi_id},sort_query=[("logtimestamp",-1)],limit=1)

        elif roi_id !="" and roi_config != "":
            if not draw_mostrecent:
                objs = self._msg_store.query(SOMAROIObject._type, message_query={"map_name": map_name,
                                                                      "id": roi_id, "config":roi_config})
            else:
                objs = self._msg_store.query(SOMAROIObject._type, message_query={"map_name": map_name,
                                                                      "id": roi_id, "config":roi_config},sort_query=[("logtimestamp",-1)],limit=1)

        else:
            if not draw_mostrecent:
                objs = self._msg_store.query(SOMAROIObject._type, message_query={"map_name": map_name,
                                                                      "config":roi_config})
            else:
                objs = self._msg_store.query(SOMAROIObject._type, message_query={"map_name": map_name,
                                                                      "config":roi_config},sort_query=[("logtimestamp",-1)],limit=1)



        return objs

    def load_objects(self, map_name, roi_id, roi_config, draw_all,draw_mostrecent):

        self._delete_markers()

	    # this is the array for roi ids
        self._soma_obj_roi_ids = dict()

        markerarray = MarkerArray()

        #get objects from db
        objs = self._retrieve_objects(map_name,roi_id,roi_config,draw_all,draw_mostrecent)

        # if collection is empty return False
        if not objs:
            return False
        count = 1
        # otherwise, load all object from collection
        for o,om  in objs:
            for pose in o.posearray.poses:
                self.load_object(o.id, o.type, pose,count,markerarray)
                count +=1
            self.draw_roi(roi_id,o.posearray.poses,markerarray,count)
            count +=1

        self.markerpub.publish(markerarray)
        return True

    def draw_roi(self, roi,poses,markerarray,ccstart):

        roicp = roi

        p = poses
        cc = ccstart

        for pose in p:
            int_marker = self.create_roi_marker(roi, pose, p,cc)
            markerarray.markers.append(int_marker)
            cc = cc+1


    def load_object(self, soma_id, soma_type, pose,markerno, markerarray):

        int_marker = self.create_object_marker(soma_id, soma_type, pose, markerno)

        markerarray.markers.append(int_marker)



    def create_object_marker(self, soma_obj, soma_type, pose, markerno):

	# create an interactive marker for our server
        marker = Marker()
        marker.header.frame_id = "map"
        marker.pose = pose
        marker.id = markerno;

        marker.pose.position.z = 0.01


        marker.type = Marker.SPHERE
        marker.action = 0
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.pose.position.z = (marker.scale.z / 2)

        random.seed(soma_type)
        val = random.random()
        marker.color.r = r_func(val)
        marker.color.g = g_func(val)
        marker.color.b = b_func(val)
        marker.color.a = 1.0
        #marker.pose = pose

        return marker

   # This part draws the line strips between the points
    def create_roi_marker(self, roi, pose, points, count):

        #points are all the points belong to that roi, pose is one of the points
        marker = Marker()

       # print "Marker name: ", int_marker.name

        marker.pose = pose
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.1
        marker.id= count

        #random.seed(10)
        #val = random.random()
        marker.color.r = self.rgb[0]#r_func(val)
        marker.color.g = self.rgb[1]#g_func(val)
        marker.color.b = self.rgb[2]#b_func(val)
        marker.color.a = 1.0

        marker.points = []
        for point in points:
            p = Point()
            pose = point

            p.x = pose.position.x - marker.pose.position.x
            p.y = pose.position.y - marker.pose.position.y
            marker.points.append(p)

        p = Point()
        pose = points[0]
        p.x = pose.position.x - marker.pose.position.x
        p.y = pose.position.y - marker.pose.position.y
        marker.points.append(p)

        return marker





if __name__=="__main__":


    rospy.init_node("soma_roi_drawer")
    rospy.loginfo("Running SOMA ROI Drawer")
    SOMAROIDrawer()
