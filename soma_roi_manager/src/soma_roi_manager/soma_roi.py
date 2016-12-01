#!/usr/bin/env python

import roslib; roslib.load_manifest("soma_roi_manager")
import rospy
from rospkg import RosPack
import json
import argparse
import random
import copy
import sys
import math

from threading import Timer

from mongodb_store.message_store import MessageStoreProxy
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

from soma_map_manager.srv import *

from soma_msgs.msg import SOMAROIObject
from soma_msgs.msg import SOMAOccupancyMap
from bson.objectid import ObjectId
from soma_manager.srv import SOMAQueryROIs

from datetime import datetime

def trapezoidal_shaped_func(a, b, c, d, x):
    min_val = min(min((x - a)/(b - a), float(1.0)), (d - x)/(d - c))
    return max(min_val, float(0.0))

def coords_to_lnglat(x, y):
        earth_radius = 6371000.0 # in meters
        lng = 90 - math.degrees(math.acos(float(x) / earth_radius))
        lat = 90 - math.degrees(math.acos(float(y) / earth_radius))
        return [lng , lat]


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


class SOMAROIManager():

    def __init__(self, soma_conf, config_file=None, db_name ="somadata", collection_name="roi"):

        self.map_unique_id = -1
        self.soma_conf = soma_conf
        if config_file:
            self._config_file = config_file
        else:
            # default file
            rp = RosPack()
            path = rp.get_path('soma_roi_manager') + '/config/'
            filename = 'default.json'
            self._config_file=path+filename

        self._soma_obj_ids = dict()
        self._soma_obj_msg = dict()
        self._soma_obj_soma_ids = dict()
        self._soma_obj_type = dict()
        self._soma_obj_pose = dict()
        self._soma_obj_markers = dict()

        self.db_name = db_name
        self.collection_name = collection_name

        self._interactive = True

        self._msg_store=MessageStoreProxy(database=self.db_name, collection=self.collection_name)

        self._server = InteractiveMarkerServer("soma_roi/%s" % soma_conf)

        rospy.loginfo("Running SOMA ROI Manager (conf: %s, types: %s db_name: %s collection_name: %s)", self.soma_conf, config_file,self.db_name,self.collection_name)

        self._init_types()

        # Get the SOMA map name and unique id
        resp = self._init_map()
        if resp == None:
            rospy.logerr("No map info received. Quitting SOMA ROI manager...")
            return
        if(self._check_soma_roi_queryservice() == None):
            rospy.logerr("SOMA ROI Query service is not active. Quitting SOMA ROI manager...")
            return
        self.soma_map_name = resp.map_name
        self.map_unique_id = resp.map_unique_id
        str_msg = "Map name: ",self.soma_map_name," Unique ID: ",self.map_unique_id
        rospy.loginfo(str_msg)

        self._init_menu()

        self.load_objects()

    def _check_soma_roi_queryservice(self):
        rospy.loginfo("SOMA ROI Manager is waiting for the SOMA query ROI service...")
        try:
            rospy.wait_for_service('soma/query_rois',timeout=30)
            return True
        except rospy.ROSException, e:
            print "Service call failed: %s"%e
            return None
    # Initialize map
    def _init_map(self):
        rospy.loginfo("SOMA ROI Manager is waiting for the SOMA map info...")
        try:
            rospy.wait_for_service('soma/map_info',timeout=30)
        except:
            return None
        try:
           map_info = rospy.ServiceProxy('soma/map_info',MapInfo)
           resp1 = map_info(0)
           return resp1
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e
           return None

    # Read config file
    def _init_types(self):


        with open(self._config_file) as config_file:
            config = json.load(config_file)

            self.mesh = dict()
            for k, v in config['roi'].iteritems():
                self.mesh[k] = v

    ## Initialize the right-click menu
    def _init_menu(self):

        self.menu_handler = MenuHandler()

        add_point_entry = self.menu_handler.insert( "Add Point", callback=self._add_point_cb)
        del_point_entry = self.menu_handler.insert( "Delete Point", callback=self._del_point_cb)


        add_entry = self.menu_handler.insert( "Add ROI" )

        self.menu_item = dict()
        for k in sorted(self.mesh.keys()):
            entry =  self.menu_handler.insert(k, parent=add_entry, callback=self._add_cb)
            self.menu_item[entry] = k

        del_entry =  self.menu_handler.insert( "Delete ROI", callback=self._del_cb)


        enable_entry = self.menu_handler.insert( "Movement control", callback=self._enable_cb )
        self.menu_handler.setCheckState( enable_entry, MenuHandler.CHECKED )

    # Add ROI callback
    def _add_cb(self, feedback):
        rospy.loginfo("Add ROI: %s", self.menu_item[feedback.menu_entry_id])

        # Add minimum 3 vertices for a valid ROI
        pose = copy.deepcopy(feedback.pose)

        pose.position.x +=1.0
        pose.position.y +=1.0

        self.add_roi(self.menu_item[feedback.menu_entry_id], pose)
        latest_soma_roi = self._soma_obj_msg[str(self._soma_id)]

        self.draw_roi(latest_soma_roi)

    # Delete ROI callback
    def _del_cb(self, feedback):
        rospy.loginfo("Delete ROI: %s", feedback.marker_name)
        roi_and_index = feedback.marker_name.split('_')
        roi = roi_and_index[0]
        rospy.loginfo("ROI Info %s",roi)
        self.delete_object(roi,feedback.marker_name,True)

    # Add Vertex callback
    def _add_point_cb(self, feedback):

        #This is the object that we are pressing (feedback) so
        #that we can get the marker name etc..
        rospy.loginfo("Add point from marker: %s", feedback.marker_name)

        roi_and_index = feedback.marker_name.split('_')

        #This is the roi that we are adding the vertex to
        soma_roi = self._soma_obj_msg[roi_and_index[0]]

        # This is the type of the roi (Office, Library, etc..)
        t   = self._soma_obj_type[roi_and_index[0]]

        # Get the pose and create the new object a little away
        pose = feedback.pose
        pose.position.x = pose.position.x+1.0*math.cos(math.radians(90))
        pose.position.y = pose.position.y+1.0*math.cos(math.radians(45))
        ######################################################

        # Add object
        self.add_object(t, pose, soma_roi.id)

        # Draw the ROI
        self.draw_roi(soma_roi)

    # Delete Vertex callback
    def _del_point_cb(self, feedback):

        rospy.loginfo("Delete point: %s", feedback.marker_name)

        roi_and_index = feedback.marker_name.split('_')

        # Find out which marker wants to be deleted
        markerindex = roi_and_index[1]
        roi = roi_and_index[0]

        marker = self._soma_obj_markers[roi][markerindex]
        keys = self._soma_obj_markers[roi].keys();

        if(len(keys) == 3):
            rospy.logerr("The minimum number of vertices is 3. You cannot delete more vertices!!")
            return

        #this was the last marker of this roi so we should delete the roi
        if(len(keys)==1):
            self.delete_object(roi,feedback.marker_name,True)
            return

        # We only want to delete particular marker
        del self._soma_obj_markers[roi][markerindex]

        # We delete the particular pose of that marker
        del self._soma_obj_pose[roi][markerindex]

        self.delete_object(roi,feedback.marker_name,False)

        soma_roi = self._soma_obj_msg[roi]

        self.draw_roi(soma_roi)

    def _update_poly(self, feedback):
        return

    def _update_cb(self, feedback):

        p = feedback.pose.position

        roi_and_index = feedback.marker_name.split('_')

        roi = roi_and_index[0]
        markerindex = roi_and_index[1]

        self._soma_obj_markers[roi][markerindex].pose = feedback.pose

        self._soma_obj_pose[roi_and_index[0]][markerindex] = feedback.pose

        self.draw_roi(self._soma_obj_msg[roi])


        if hasattr(self, "vp_timer_"+roi):
            getattr(self, "vp_timer_"+roi).cancel()
        setattr(self, "vp_timer_"+roi,
                Timer(5, self.update_object, [roi]))
        getattr(self, "vp_timer_"+roi).start()

    def _enable_cb(self, feedback):

        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )

        if state == MenuHandler.CHECKED:
            self.menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            self._interactive = False
        else:
            self.menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            self._interactive = True

        self.menu_handler.reApply( self._server )

        self.load_objects()

        self._server.applyChanges()

    def _next_id(self):
        self._soma_id += 1
        return self._soma_id


    # Retrieve the objects from DB
    def _retrieve_objects(self):

        query_service = rospy.ServiceProxy('soma/query_rois',SOMAQueryROIs)
        resp = query_service(query_type=0,roiconfigs=[self.soma_conf],returnmostrecent=True)#self._msg_store.query(SOMAROIObject._type, message_query={"map_name": self.soma_map_name, "config":self.soma_conf, "returnmostrecent":True})

        max_id = 0

        ids = []

        for o in resp.rois:
            ''' Store ids '''
            ids.append(o.id)

            if int(o.id) > max_id:
                max_id = int(o.id)

        self._soma_id = max_id

        returnedobjs = []
        returnedmetas = []

        ''' Get the latest state of each roi  '''
        for i,o in enumerate(resp.rois):
            #objs = self._msg_store.query(SOMAROIObject._type, message_query={"map_name": self.soma_map_name, "config":self.soma_conf, "id":i},sort_query=[("logtimestamp",-1)])
            returnedobjs.append(o)
            returnedmetas.append(resp.unique_ids[i])

        ''' Return latest states '''
        return zip(returnedobjs,returnedmetas)

    def load_objects(self):

        #get objects from db
        objs = self._retrieve_objects()

        # if collection is empty insert initial object
        if not objs:

            pose = Pose()
            pose.position.x = 0
            pose.position.y = 0

            self.add_roi('Office',pose)

            # Draw the ROI
            self.draw_roi(self._soma_obj_msg['1'])

            return

        # Otherwise, load all object from collection
        for o,om  in objs:
#
            #print om
            self._soma_obj_ids[o.id] = om

            self._soma_obj_msg[o.id] = o

            self._soma_obj_type[o.id] = o.type
            self._soma_obj_pose[o.id] = dict()

            self._soma_obj_markers[o.id] = dict()

            for pose in o.posearray.poses:
                self.load_object(o.id, o.type, pose)

        self.draw_all_roi()

    def draw_all_roi(self):

        for key  in self._soma_obj_msg:
            self.draw_roi(self._soma_obj_msg[key])

    def undraw_all_roi(self):

        for key  in self._soma_obj_msg:
            self.undraw_roi(self._soma_obj_msg[key])

    def sort_marker_positions(self,posesdict):
        sortedposes = list()

        pointkeys = posesdict.keys()

        pointkeys.sort(key=int)

        for akey in pointkeys:
            sortedposes.append(posesdict[akey])

        return sortedposes

    def draw_roi(self, soma_roi):
        #print soma_roi
        t = soma_roi.type

        p = self.sort_marker_positions(self._soma_obj_pose[soma_roi.id])

        current_index = 0
        for pose in p:
            int_marker = self.create_roi_marker(soma_roi.id, t, pose, p, current_index)
            self._server.erase("ROI-" + soma_roi.id)
            self._server.applyChanges()
            self._server.insert(int_marker, self._update_poly)
            self._server.applyChanges()
            current_index +=1


    def undraw_roi(self, roi):
        self._server.erase("ROI-" + roi.id)
        self._server.applyChanges()

    def load_object(self, soma_id, soma_type, pose):

        #default marker count value
        markerno = 1

        #get the max key value
        if self._soma_obj_markers[str(soma_id)].keys():
            maxkey = max(self._soma_obj_markers[str(soma_id)].keys(), key=int)
            markerno = int(maxkey)+1;

        self._soma_obj_pose[str(soma_id)][str(markerno)] = pose

        int_marker = self.create_object_marker(soma_id, soma_type, pose, markerno)

        self._soma_obj_markers[str(soma_id)][str(markerno)] = int_marker

        #call the update_cb when marker moves
        self._server.insert(int_marker, self._update_cb)

        name = soma_id+'_'+str(markerno)

        self.menu_handler.apply( self._server, name )

        self._server.applyChanges()

    def add_roi(self, soma_type, anchor_pose, soma_id=None):

        #create a SOMAROI Object
        soma_obj = SOMAROIObject()
        #print soma_id
        # a new roi
        if soma_id == None:

            #soma_id is an id for the soma object like 1,2,3,4. It updates itself from the db if there are existing objects
            soma_id = self._next_id()
            self._soma_id = soma_id
            soma_obj.id = str(soma_id)
           # print soma_obj.id

            soma_obj.map_name = str(self.soma_map_name)
            soma_obj.map_unique_id = str(self.map_unique_id)

            soma_obj.config = str(self.soma_conf)

            soma_obj.type = soma_type
            soma_obj.posearray.poses.append(anchor_pose)
            soma_obj.header.frame_id = '/map'
            soma_obj.header.stamp = rospy.get_rostime()

            self.insert_soma_time_fields(soma_obj)

            #print dt.day, dt.hour, dt.minute
            #self._soma_obj_roi_ids[str(soma_roi_id)] = list()
            self._soma_obj_markers[soma_obj.id] = dict()
            self._soma_obj_pose[soma_obj.id] = dict()

            self._soma_obj_pose[soma_obj.id][str(1)] = anchor_pose

            self.load_object(str(soma_id),soma_type,anchor_pose)

            for i in range(2,4):
                p = Pose()
                p.position.x = anchor_pose.position.x + 1.0*i*math.cos(math.radians(90*i))
                p.position.y = anchor_pose.position.y + 1.0*i*math.cos(math.radians(120*i))
                self.load_object(str(soma_id),soma_type,p)
                self._soma_obj_pose[soma_obj.id][str(i)] = p
                soma_obj.posearray.poses.append(p)

            # If we have at least 3 vertices add geojson part and a new object
            if len(soma_obj.posearray.poses) >=3:
                self.insert_geo_poses(soma_obj)
                #print soma_obj
            try:
                _id = self._msg_store.insert(soma_obj)
                self._soma_obj_ids[soma_obj.id] = _id
                self._soma_obj_type[soma_obj.id] = soma_type
                self._soma_obj_msg[soma_obj.id] = soma_obj
            except:
                soma_obj.geotype = ''
                soma_obj.geoposearray = []
                rospy.logerr("The polygon of %s %s is malformed (self-intersecting) => Please update geometry." % (soma_obj.type, soma_obj.id))



    #soma_type = Office, Kitchen, etc, Pose is position
    def add_object(self, soma_type, pose, soma_id=None):

        #create a SOMAROI Object
        msg = self._soma_obj_msg[soma_id]
        _id = self._soma_obj_ids[soma_id]

        new_msg = copy.deepcopy(msg)

        #call the object with that id
        #res = self._msg_store.query(SOMAROIObject._type,message_query={'id':str(soma_id)},sort_query=[("logtimestamp",-1)])
        #res = self._msg_store.query(SOMAROIObject._type,message_query={'id':str(soma_id)})
        res = self._msg_store.query(SOMAROIObject._type,message_query={'id':str(soma_id),'config':self.soma_conf})

        #iterate through the objects.
        #for o,om in res:
	   # print o
        soma_obj = copy.deepcopy(res[0][0])
	    #break

        if soma_obj:

            self.load_object(str(soma_id), soma_type, pose)

            soma_obj.posearray.poses = self.sort_marker_positions(self._soma_obj_pose[soma_obj.id])

            self.insert_geo_poses(soma_obj)

            self.insert_soma_time_fields(soma_obj)

            try:
                if(len(res) == 1):
                    val =  int(rospy.get_rostime().to_sec()) & 0xffffffff-res[0][0].logtimestamp
                    if(val <= 90):
                        self._msg_store.update_id(_id, soma_obj)
                        return
                _id = self._msg_store.insert(soma_obj)

                self._soma_obj_ids[soma_obj.id] = _id

                rospy.loginfo("ROI Store: updated roi (%s %s)" % (soma_obj.type, soma_obj.id) )

            except:

                soma_obj.geotype = ''
                soma_obj.geoposearray = []
                rospy.logerr("The polygon of %s %s is malformed (self-intersecting) => Please update geometry." % (soma_obj.type, soma_obj.id))

            self._soma_obj_msg[soma_obj.id] = soma_obj




    def insert_soma_time_fields(self,soma_obj,timestamp=None):

        if timestamp == None:

            #convert to uint32
            soma_obj.logtimestamp = int(rospy.get_rostime().to_sec()) & 0xffffffff

            dt = datetime.utcfromtimestamp(soma_obj.logtimestamp)
            soma_obj.loghour = int(dt.hour)
            soma_obj.logminute = int(dt.minute)
            soma_obj.logday = int(dt.weekday())+1
            soma_obj.logtimeminutes = int(dt.hour)*60 + int(dt.minute)
        else:
            #convert to uint32
            soma_obj.logtimestamp = timestamp & 0xffffffff

            dt = datetime.utcfromtimestamp(soma_obj.logtimestamp)
            soma_obj.loghour = int(dt.hour)
            soma_obj.logminute = int(dt.minute)
            soma_obj.logday = int(dt.weekday())
            soma_obj.logtimeminutes = int(dt.hour)*60 + int(dt.minute)


    def delete_object(self, soma_id, marker_name, should_delete_roi):

        _id = self._soma_obj_ids[soma_id]
        msg = self._soma_obj_msg[soma_id]

        if(should_delete_roi):
            try:
                '''If we have a update timer associated with the roi, cancel it'''
                if hasattr(self, "vp_timer_"+soma_id):
                    getattr(self, "vp_timer_"+soma_id).cancel()
                # Get all the history object with 'id'
                res = self._msg_store.query(SOMAROIObject._type,message_query={'id':str(soma_id),'config':self.soma_conf})
                #iterate through the objects. Delete all the history
                for o,om in res:
                    soma_obj = o
                    self._msg_store.delete(str(om["_id"]))
            except:
                rospy.logerr("Error deleting ROI %s." % (str(soma_id)) )
                return

            del self._soma_obj_ids[soma_id]
            markers = self._soma_obj_markers[soma_id]
            for key,amarker in markers.iteritems():
                self._server.erase(amarker.name)
                self._server.applyChanges()
            self.undraw_roi(self._soma_obj_msg[soma_id])
            del self._soma_obj_msg[soma_id]
            del self._soma_obj_markers[soma_id]
            del self._soma_obj_pose[soma_id]
            return

        new_msg = copy.deepcopy(msg)

        new_msg.posearray.poses = self.sort_marker_positions(self._soma_obj_pose[str(soma_id)])

        self.insert_geo_poses(new_msg)
        self.insert_soma_time_fields(new_msg)
        try:
            ## if this is the first time the region has been modified, we have a timeout period of 90 seconds before inserting a new version of the roi
            res = self._msg_store.query(SOMAROIObject._type,message_query={'id':str(soma_id),'config':self.soma_conf})

            if(len(res) == 1):
                val =  (int(rospy.get_rostime().to_sec()) & 0xffffffff)-res[0][0].logtimestamp

                if(val <= 90):
                    self._msg_store.update_id(_id, new_msg)
                    return

            _id = self._msg_store.insert(new_msg)

            self._soma_obj_ids[new_msg.id] = _id
            rospy.loginfo("Marker %s updated successfully" %(marker_name))
            self._server.erase(marker_name)
            self._server.applyChanges()
        except:
            rospy.logerr("Error deleting Marker %s." % (marker_name) )





    def update_object(self, roi):
        rospy.loginfo("Updated roi: %s", roi)

        _id = self._soma_obj_ids[roi]
        msg = self._soma_obj_msg[roi]

        new_msg = copy.deepcopy(msg)

        new_msg.posearray.poses = self.sort_marker_positions(self._soma_obj_pose[roi])
        self.insert_geo_poses(new_msg)
        self.insert_soma_time_fields(new_msg)

        try:
            ## if this is the first time the region has been modified, we have a timeout period of 90 seconds before inserting a new version of the roi
            res = self._msg_store.query(SOMAROIObject._type,message_query={'id':str(roi),'config':self.soma_conf})

            if(len(res) == 1):
                val =  (int(rospy.get_rostime().to_sec()) & 0xffffffff)-res[0][0].logtimestamp
                #print val
                if(val <= 90):
                    self._msg_store.update_id(_id, new_msg)
                    return

            _id = self._msg_store.insert(new_msg)
            #print _id
            self._soma_obj_ids[new_msg.id] = _id
            rospy.loginfo("ROI %s updated successfully" %(roi))
        except:
            rospy.logerr("Error updating ROI %s" %(roi))

    def insert_geo_poses(self,msg):
        coordinates = PoseArray()
        for pose in msg.posearray.poses:
            p = copy.deepcopy(pose)
            p.orientation.w = 0.0
            res = coords_to_lnglat(p.position.x, p.position.y)
            p.position.x = res[0]
            p.position.y = res[1]
            coordinates.poses.append(p)

        coordinates.poses.append(coordinates.poses[0])
        msg.geoposearray = coordinates



    def create_object_marker(self, soma_id, soma_type, pose,markerno):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = soma_id+'_'+str(markerno)
        int_marker.description = soma_type + ' (' + soma_id +'_'+str(markerno)+  ')'
        int_marker.pose = pose
        int_marker.pose.position.z = 0.01


        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        int_marker.pose.position.z = (marker.scale.z / 2)

        random.seed(soma_type)
        val = random.random()
        marker.color.r = r_func(val)
        marker.color.g = g_func(val)
        marker.color.b = b_func(val)
        marker.color.a = 1.0
        #marker.pose = pose
        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE

        if self._interactive:
            int_marker.controls.append(copy.deepcopy(control))
            # add the control to the interactive marker
            int_marker.controls.append(control);

        # add menu control
        menu_control = InteractiveMarkerControl()

        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        menu_control.always_visible = True

        menu_control.markers.append( marker) #makeBox(int_marker) )
        int_marker.controls.append(menu_control)

        return int_marker

    # This part draws the line strips between the points
    def create_roi_marker(self, soma_roi_id, soma_type, pose, points, current_index):
        #print "POINTS: " + str(points)
        #points are all the points belong to that roi, pose is one of the points
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = "ROI-" + soma_roi_id
       # print "Marker name: ", int_marker.name
        int_marker.description = soma_roi_id
        int_marker.pose = pose

        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.1

        random.seed(soma_type)
        val = random.random()
        marker.color.r = r_func(val)
        marker.color.g = g_func(val)
        marker.color.b = b_func(val)
        marker.color.a = 1.0

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append( marker )

        int_marker.controls.append(control )

        marker.points = []

        for point in points:
            p = Point()
            pose = point#self._soma_obj_pose[point]

            p.x = pose.position.x - int_marker.pose.position.x
            p.y = pose.position.y - int_marker.pose.position.y
            marker.points.append(p)

        p = Point()
        pose = points[0]
        p.x = pose.position.x - int_marker.pose.position.x
        p.y = pose.position.y - int_marker.pose.position.y
        marker.points.append(p)
        return int_marker
