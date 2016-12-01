#!/usr/bin/env python

import roslib; roslib.load_manifest("soma_manager")
import rospy
from rospkg import RosPack
import json
import argparse
import random
import copy
import sys

from threading import Timer

from mongodb_store.message_store import MessageStoreProxy
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from geometry_msgs.msg import Pose

from soma_msgs.msg import SOMAObject
from soma_manager.srv import *
from soma_map_manager.srv import *
from bson.objectid import ObjectId
from std_msgs.msg import String

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


class SOMAManager():

    def __init__(self, soma_conf, config_file=None):

        #self.soma_map = soma_map
        self.soma_conf = soma_conf
        if config_file:
            self._config_file = config_file
        else:
            # default file
            rp = RosPack()
            #rospy.loginfo("%s",rp.list())
            path = rp.get_path('soma_objects') + '/config/'
            filename = 'default.json'
            self._config_file=path+filename
        self._soma_obj_ids = dict()
        self._soma_obj_msg = dict()

        self._interactive = True

        self._msg_store=MessageStoreProxy(database="somadata", collection="object")

        #Debug purposes
        #objs = self._msg_store.query(SOMAObject._type, message_query={}, projection_query={"pose":1, "mesh":1},limit=2, sort_query=[("_id",-1)])
	    #objs = self._msg_store.query(SOMAObject._type, message_query={})
        #print objs

         # Get the SOMA map name and unique id
        resp = self._init_map()
        if resp == None:
            rospy.signal_shutdown("No map info provided...")
            return None

        self.soma_map = resp.map_name
        self.map_unique_id = resp.map_unique_id

        rospy.loginfo("Map name: %s Map Unique ID: %s",self.soma_map,self.map_unique_id)

        if(self._check_soma_insertservice() == False):
            return None

        if(self._check_soma_queryservice() == False):
            return None

        self._server = InteractiveMarkerServer("soma")

        self._init_types()

        self._init_menu()

        self.load_objects()

        rospy.spin()

    # Listens the map information from soma map_manager
    def _init_map(self):
        print "Waiting for the map info from soma_map_manager..."
        try:
            rospy.wait_for_service('soma/map_info', timeout=5)
            rospy.loginfo("SOMA map info received...")
        except:
            rospy.logerr("No SOMA map_info service!! Quitting...")
            return None
        try:
            map_info = rospy.ServiceProxy('soma/map_info',MapInfo)
            resp1 = map_info(0)
            return resp1
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s",e)
            return None

    # Checks the soma insert service information from soma data_manager
    def _check_soma_insertservice(self):
        print "Waiting for SOMA object insert service..."
        try:
            rospy.wait_for_service('soma/insert_objects', timeout=5)
            rospy.loginfo("SOMA insert service is active...")
            return True
        except:
            rospy.logerr("No SOMA insert service!! Quitting...")
            return False
    # Checks the soma insert service information from soma data_manager
    def _check_soma_queryservice(self):
        print "Waiting for SOMA object query service..."
        try:
            rospy.wait_for_service('soma/query_objects', timeout=5)
            rospy.loginfo("SOMA object query service is active...")
            return True
        except:
            rospy.logerr("No SOMA query service!! Quitting...")
            return False

    def _init_types(self):
        # read from config in soma_objects

        with open(self._config_file) as config_file:
            config = json.load(config_file)

            self.mesh = dict()
            self.marker = dict()
            if '2D' in config:
                for k, v in config['2D'].iteritems():
                    self.mesh[k] = v
                    self.marker[k] = '2D'

            if '3D' in config:
                for k, v in config['3D'].iteritems():
                    self.mesh[k] = v
                    self.marker[k] = '3D'

    def _init_menu(self):

        self.menu_handler = MenuHandler()
        add_entry = self.menu_handler.insert( "Add object" )

        self.menu_item = dict()
        for k in sorted(self.mesh.keys()):
            entry =  self.menu_handler.insert(k, parent=add_entry, callback=self._add_cb)
            self.menu_item[entry] = k

        del_entry =  self.menu_handler.insert( "Delete object", callback=self._del_cb)

        enable_entry = self.menu_handler.insert( "Movement control", callback=self._enable_cb )

        self.menu_handler.setCheckState( enable_entry, MenuHandler.CHECKED )

    def _add_cb(self, feedback):
        rospy.loginfo("Add marker: %s", self.menu_item[feedback.menu_entry_id])
        p = Pose()
        p.position.x = feedback.pose.position.x+1.0
        p.position.y = feedback.pose.position.y+1.0
        self.add_object(self.menu_item[feedback.menu_entry_id], p)

    def _del_cb(self, feedback):
        rospy.loginfo("Delete marker: %s", feedback.marker_name)
        self.delete_object(feedback.marker_name)

    def _update_cb(self, feedback):
        p = feedback.pose.position
        #print "Marker " + feedback.marker_name + " position: " + str(round(p.x,2)) + ", " + str(round(p.y,2)) +  ", " + str(round(p.z,2))

        if hasattr(self, "vp_timer_"+feedback.marker_name):
            getattr(self, "vp_timer_"+feedback.marker_name).cancel()
        setattr(self, "vp_timer_"+feedback.marker_name,
                Timer(3, self.update_object, [feedback]))
        getattr(self, "vp_timer_"+feedback.marker_name).start()

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

    def _retrieve_objects(self):

        query_objects = rospy.ServiceProxy('soma/query_objects',SOMAQueryObjs)

        resp = query_objects(query_type=0,configs=[self.soma_conf])
        if resp:
            objs = resp.objects
        else:
            return None

        max_id = 0
        for o in objs:
            if int(o.id) > max_id:
                max_id = int(o.id)
        self._soma_id = max_id

        return resp

    def load_objects(self):

        resp = self._retrieve_objects()

        objs = resp.objects
        unique_ids = resp.unique_ids

        # if collection is empty insert initial object
        if not objs:
            pose = Pose()
            self.add_object('Table', pose)
            return

        # otherwise, load all object from collection

        for i,o in enumerate(objs):
            self._soma_obj_ids[o.id] = unique_ids[i]
            self._soma_obj_msg[o.id] = o
            self.load_object(o.id, o.type, o.pose)



    def load_object(self, soma_id, soma_type, pose):

        int_marker = self.create_object_marker(soma_id, soma_type, pose)

        self._server.insert(int_marker, self._update_cb)

        self.menu_handler.apply( self._server, soma_id )

        self._server.applyChanges()

    def _insert_object_to_DB(self,object):

        insert_objects = rospy.ServiceProxy('soma/insert_objects',SOMAInsertObjs)
        objects = list()
        objects.append(object)

        try:
            resp = insert_objects(objects)
            if(resp.result == True):
                rospy.loginfo("Object inserted successfully")
            else:
                rospy.logerr("Error inserting object!! Check DB services...")
        except:
            rospy.logerr("Error inserting object!! soma/insert_objects service call failed!")

        return resp

    def add_object(self, soma_type, pose):
        # todo: add to mongodb

        soma_id = self._next_id()

        soma_obj = SOMAObject()
        soma_obj.id = str(soma_id)
        soma_obj.map_unique_id = str(self.map_unique_id)
        soma_obj.map_name = str(self.soma_map)
        soma_obj.config = str(self.soma_conf)
        soma_obj.type = soma_type
        soma_obj.pose = pose
        soma_obj.pose.position.z = 0.0

        soma_obj.mesh = str(self.mesh[soma_type])
        soma_obj.logtimestamp = rospy.Time.now().secs

        resp = self._insert_object_to_DB(soma_obj)

        if resp.result == True:
            self._soma_obj_ids[soma_obj.id] = resp.db_ids[0]
            self._soma_obj_msg[soma_obj.id] = soma_obj
            self.load_object(str(soma_id), soma_type, soma_obj.pose)



    def delete_object(self, soma_id):

        # message store
        _id = self._soma_obj_ids[str(soma_id)]

        ids = list()

        delete_object = rospy.ServiceProxy('soma/delete_objects',SOMADeleteObjs)

        ids.append(soma_id)
        try:
            resp = delete_object(ids)
            if(resp.result == True):
                self._server.erase(soma_id)
                self._server.applyChanges()
                rospy.loginfo("Object deleted successfully")
            else:
                rospy.logerr("Error deleting object!! Check DB services...")
        except:
            rospy.logerr("Error deleting object!! soma/delete_objects service call failed!")

        #self._msg_store.delete(str(_id))



    def update_object(self, feedback):
        print "Updated marker " + feedback.marker_name

        _id = self._soma_obj_ids[feedback.marker_name]
        msg = self._soma_obj_msg[feedback.marker_name]

        new_msg = copy.deepcopy(msg)
        new_msg.pose = feedback.pose

        st = String()
        st.data = '%s'%str(_id)

        update_object = rospy.ServiceProxy('soma/update_object',SOMAUpdateObject)
        try:
            resp = update_object(str(_id),new_msg)
            if resp.result == True:
                rospy.loginfo("Object updated successfully")
            else:
                rospy.logerr("Error updating object!! Check DB services...")
        except:
            rospy.logerr("Error updating object!! soma/update object service call failed!")
        #print resp

    def create_object_marker(self, soma_obj, soma_type, pose):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = soma_obj
        int_marker.description = "id" + soma_obj
        int_marker.pose = pose
        if int_marker.pose.position.z == 0.0:
            int_marker.pose.position.z = 0.01

        mesh_marker = Marker()
        mesh_marker.type = Marker.MESH_RESOURCE
        mesh_marker.scale.x = 1
        mesh_marker.scale.y = 1
        mesh_marker.scale.z = 1

        random.seed(soma_type)
        val = random.random()
        mesh_marker.color.r = r_func(val)
        mesh_marker.color.g = g_func(val)
        mesh_marker.color.b = b_func(val)
        mesh_marker.color.a = 1.0
        #mesh_marker.pose = pose
        mesh_marker.mesh_resource = self.mesh[soma_type]

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE


        if self._interactive:
            int_marker.controls.append(copy.deepcopy(control))
            # add the control to the interactive marker
            if self.marker[soma_type] == '3D':
                control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
                int_marker.controls.append(control)

        # add menu control
        menu_control = InteractiveMarkerControl()

        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        menu_control.always_visible = True

        menu_control.markers.append( mesh_marker) #makeBox(int_marker) )
        int_marker.controls.append(menu_control)

        return int_marker


if __name__=="__main__":

    # TODO: add list command

    parser = argparse.ArgumentParser(prog='soma.py')
    #parser.add_argument("map", nargs=1, help='Name of the used 2D map')
    parser.add_argument("conf", nargs=1, help='Name of the object configuration')
    parser.add_argument('-t', metavar='config-file')

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("soma_obj_manager")
    rospy.loginfo("Running SOMA Manual Object Manager ( conf: %s, types: %s)", args.conf[0], args.t)
    SOMAManager(args.conf[0],args.t)
