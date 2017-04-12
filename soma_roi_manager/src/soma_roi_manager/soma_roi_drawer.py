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
from soma_manager.srv import SOMAQueryROIs


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
        self.markerarray = MarkerArray()
        #self.rgb[0] = 0.0
        #self.rgb[1] = 0.0
        #self.rgb[2] = 1.0

        self._msg_store=MessageStoreProxy(database="somadata",collection="roi")

        s = rospy.Service('soma/draw_roi', DrawROI, self.handle_draw_roi)

       # Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
        self.markerpub = rospy.Publisher("soma_roi_marker_array", MarkerArray, queue_size=1)

        rospy.spin()

    def handle_draw_roi(self,req):
        #print req
        if(len(req.rgb)==3):
            self.rgb = req.rgb
        self._delete_markers()

        for roi_config in req.roi_configs:
            if not self.load_objects(req.roi_id,roi_config,req.draw_all,req.draw_mostrecent):
                DrawROIResponse(False)
                return False

        self.markerpub.publish(self.markerarray)
        DrawROIResponse(True)
        return True

        #return True


    def _delete_markers(self):
        marker = Marker()
        marker.action = 3
        marker.header.frame_id = "map"
        self.markerarray = MarkerArray()
        markerarray = MarkerArray()
        markerarray.markers.append(marker)

        self.markerpub.publish(markerarray)

    def coords_to_lnglat(x, y):
        earth_radius = 6371000.0 # in meters
        lng = 90 - math.degrees(math.acos(float(x) / earth_radius))
        lat = 90 - math.degrees(math.acos(float(y) / earth_radius))
        return [lng , lat]

    def _retrieve_objects(self, roi_id, roi_config, draw_all,draw_mostrecent):
        query_service = rospy.ServiceProxy('soma/query_rois',SOMAQueryROIs)
        #self._msg_store.query(SOMAROIObject._type, message_query={"map_name": self.soma_map_name, "config":self.soma_conf, "returnmostrecent":True})

        if draw_all:
            #objs = self._msg_store.query(SOMAROIObject._type, message_query={"map_name": map_name, })
            resp = query_service(query_type=0,returnmostrecent=True)
            return resp.rois
        if roi_config == "":
            if not draw_mostrecent:
                resp = query_service(query_type=0,roiids=[roi_id],returnmostrecent=False)
                #objs = self._msg_store.query(SOMAROIObject._type, message_query={"map_name": map_name,
                #                                                      "id": roi_id})
            else:
                resp = query_service(query_type=0,roiids=[roi_id],returnmostrecent=True)
                #objs = self._msg_store.query(SOMAROIObject._type, message_query={"map_name": map_name,
                #                                                      "id": roi_id},sort_query=[("logtimestamp",-1)],limit=1)

        elif roi_id !="" and roi_config != "":
            if not draw_mostrecent:
                resp = query_service(query_type=0,roiids=[roi_id],roiconfigs=[roi_config],returnmostrecent=False)
                #objs = self._msg_store.query(SOMAROIObject._type, message_query={"map_name": map_name,
                #                                                      "id": roi_id, "config":roi_config})
            else:
                resp = query_service(query_type=0,roiids=[roi_id],roiconfigs=[roi_config],returnmostrecent=True)
                #objs = self._msg_store.query(SOMAROIObject._type, message_query={"map_name": map_name,
                #                                                      "id": roi_id, "config":roi_config},sort_query=[("logtimestamp",-1)],limit=1)

        else:
            if not draw_mostrecent:
                #objs = self._msg_store.query(SOMAROIObject._type, message_query={"map_name": map_name,
                #                                                      "config":roi_config})
                resp = query_service(query_type=0,roiconfigs=[roi_config],returnmostrecent=False)
            else:
                resp = query_service(query_type=0,roiconfigs=[roi_config],returnmostrecent=True)
                #objs = self._msg_store.query(SOMAROIObject._type, message_query={"map_name": map_name,
                #                                                      "config":roi_config},sort_query=[("logtimestamp",-1)],limit=1)


        #print len(resp.rois)
        return resp.rois

    def load_objects(self, roi_id, roi_config, draw_all,draw_mostrecent):

        #self._delete_markers()

        #get objects from db
        objs = self._retrieve_objects(roi_id,roi_config,draw_all,draw_mostrecent)

        # if collection is empty return False
        if not objs:
            return False

        count = len(self.markerarray.markers)
        # otherwise, load all object from collection
        for o in objs:
            self.draw_roi(o.config,o.posearray.poses,count)
            #count +=1


        return True

    def draw_roi(self,roi_config,poses,marker_count):

        p = poses
        marker_count = len(self.markerarray.markers)
        for pose in p:
            int_marker = self.create_object_marker(roi_config, pose, marker_count)
            self.markerarray.markers.append(int_marker)
            marker_count +=1
            #print marker_count
            int_marker = self.create_roi_marker(pose, p,marker_count)
            self.markerarray.markers.append(int_marker)
            marker_count +=1


    def create_object_marker(self, roi_config, pose, markerno):

	# create an interactive marker for our server
        marker = Marker()
        marker.header.frame_id = "map"
        marker.pose = pose
        marker.id = markerno;
        #marker.name = roi_config

        marker.pose.position.z = 0.01


        marker.type = Marker.SPHERE
        marker.action = 0
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.pose.position.z = (marker.scale.z / 2)

        random.seed(roi_config)
        val = random.random()
        marker.color.r = r_func(val)
        marker.color.g = g_func(val)
        marker.color.b = b_func(val)
        marker.color.a = 1.0
        #marker.pose = pose

        return marker

   # This part draws the line strips between the points
    def create_roi_marker(self, pose, points, count):

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
