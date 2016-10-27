#!/usr/bin/env python

import roslib; roslib.load_manifest("soma_manager")
import rospy
from rospkg import RosPack
import json
import argparse
import random
import copy
import sys
import datetime
import time
import math

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from threading import Timer
from mongodb_store.message_store import MessageStoreProxy
from soma_msgs.msg import SOMAROIObject, SOMAObject
from soma_manager.srv import *
from soma_map_manager.srv import *
from std_msgs.msg import String
from soma_manager.msg import *


# SOMA Data Manager For storing and deleting data
class SOMADataManager():

    def __init__(self, db_name="somadata", collection_name="object"):

       # self.soma_map_name = soma_map_name
        self._db_name = db_name
        self._collection_name = collection_name

        # Get the map information from soma map_manager
        resp = self._init_map()
	if resp == None:
		rospy.signal_shutdown("No map info provided...")
		return None
        #print resp
        self.map_name = resp.map_name
        self.map_unique_id = resp.map_unique_id
        rospy.loginfo("Map name: %s Map unique ID: %s",self.map_name, self.map_unique_id)

        # Initialize the mongodb proxy
        self._message_store = MessageStoreProxy(database=db_name, collection=collection_name)

        # Object insertion service
        inss = rospy.Service('soma/insert_objects', SOMAInsertObjs, self.handle_insert_request)

        # Object deletion service
        dels = rospy.Service('soma/delete_objects',SOMADeleteObjs, self.handle_delete_request)

        #Object update service
        upts = rospy.Service('soma/update_object',SOMAUpdateObject,self.handle_update_request)

        self.new_objects_pub = rospy.Publisher('soma/new_objects_inserted',SOMANewObjects,queue_size=5)

        rospy.spin()

    # Listens the map information from soma map_manager
    def _init_map(self):
        print "SOMA Data Manager is waiting for SOMA Map Service..."
        try:
            rospy.wait_for_service('soma/map_info', timeout = 30)
            #rospy.loginfo("SOMA Map Info received...")
        except:
            rospy.logerr("No 'soma/map_info' service, Quitting...")
            return None
        try:
           map_info = rospy.ServiceProxy('soma/map_info',MapInfo)
           resp1 = map_info(0)
           return resp1
        except rospy.ServiceException, e:
           rospy.logerr("Service call failed: %s"%e)
           return None

    # Handles the soma objects to be inserted
    def handle_insert_request(self,req):
        _ids = []
        obj_ids = []
        for obj in req.objects:

          if(obj.logtimestamp == 0):
            obj.logtimestamp = rospy.Time.now().secs

          d = datetime.datetime.utcfromtimestamp(obj.logtimestamp)
          obj.loghour = d.hour
          obj.logminute = d.minute
          obj.logday = d.isoweekday()
          obj.logtimeminutes = obj.loghour*60 + obj.logminute
          obj_ids.append(obj.id)

          if (obj.header.frame_id == ""):
              obj.header.frame_id = "/map"

          if(obj.cloud.header.frame_id == ""):
              obj.cloud.header.frame_id = "/map"

          obj.map_name = self.map_name
          obj.map_unique_id = self.map_unique_id

          # SOMA Objects are represented as a 3D point in the world so this could be set here as point
          obj.geotype = "Point"


          res = self.coords_to_lnglat(obj.pose.position.x,obj.pose.position.y)

          geopose = Pose()

          geopose.position.x = res[0]
          geopose.position.y = res[1]

          obj.geoposearray.poses.append(geopose)

          try:
                _id = self._message_store.insert(obj)

                _ids.append(str(_id))

          except:
                return SOMAInsertObjsResponse(False,_ids)

        msg = SOMANewObjects()
        msg.ids = obj_ids
        self.new_objects_pub.publish(msg)
        return SOMAInsertObjsResponse(True,_ids)


    # Handles the delete request of soma objs
    def handle_delete_request(self,req):

        for oid in req.ids:
            res = self._message_store.query(SOMAObject._type,message_query={"id": oid})
            #print len(res)
            for o,om in res:
                try:
                    self._message_store.delete(str(om['_id']))
                except:
                      return SOMADeleteObjsResponse(False)

        return SOMADeleteObjsResponse(True)

    # Handles the soma objects to be inserted
    def handle_update_request(self,req):

        obj = req.object


        if(obj.logtimestamp == 0):
            obj.logtimestamp = rospy.Time.now().secs

        d = datetime.datetime.utcfromtimestamp(obj.logtimestamp)
        obj.loghour = d.hour
        obj.logminute = d.minute
        obj.logday = d.isoweekday()
        obj.logtimeminutes = obj.loghour*60 + obj.logminute

        if (obj.header.frame_id == ""):
            obj.header.frame_id = "/map"

        if(obj.cloud.header.frame_id == ""):
            obj.cloud.header.frame_id = "/map"

        obj.map_name = self.map_name
        obj.map_unique_id = self.map_unique_id

        # SOMA Objects are represented as a 3D point in the world so this could be set here as point
        obj.geotype = "Point"


        res = self.coords_to_lnglat(obj.pose.position.x,obj.pose.position.y)

        geopose = Pose()

        geopose.position.x = res[0]
        geopose.position.y = res[1]

        obj.geoposearray.poses.append(geopose)

        try:
            self._message_store.update_id(req.db_id, obj)
        except:
            return SOMAUpdateObjectResponse(False)

        return SOMAUpdateObjectResponse(True)

    def coords_to_lnglat(self, x, y):
        earth_radius = 6371000.0 # in meters
        lng = 90 - math.degrees(math.acos(float(x) / earth_radius))
        lat = 90 - math.degrees(math.acos(float(y) / earth_radius))
        return [lng , lat]
