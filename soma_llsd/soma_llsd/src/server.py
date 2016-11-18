#!/usr/bin/env python

import roslib
import rospy
import sys
import argparse
import os
import uuid
import tf
import std_msgs
import pymongo
from collections import deque
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, JointState
from mongodb_store.message_store import MessageStoreProxy
from soma_llsd_msgs.msg import Segment,Observation,Scene
from geometry_msgs.msg import Pose,Point,Quaternion
from soma_llsd.srv import *

class StoreController():
    def __init__(self, db_name="somadata",scene_store_name="llsd_scene_store",segment_store_name="llsd_segment_store"):
        rospy.init_node('soma_llsd_services', anonymous = False)
        rospy.loginfo("LLSD: SOMa LLSD setting up services")
        self.scene_store = MessageStoreProxy(database=db_name, collection=scene_store_name)
        self.segment_store = MessageStoreProxy(database=db_name, collection=segment_store_name)
        rospy.loginfo("LLSD: Done!")
        rospy.loginfo("LLSD: Using database: " + db_name + ", scenes being stored at: " + scene_store_name + " segments being stored at " + segment_store_name)

        get_scene = rospy.Service('/soma_llsd/get_scene',GetScene,self.get_scene_cb)
        insert_scene = rospy.Service('/soma_llsd/insert_scene',InsertScene,self.insert_scene_cb)
        auto_insert_scene = rospy.Service('/soma_llsd/insert_scene_auto',InsertSceneAuto,self.insert_scene_auto_cb)
        update_scene = rospy.Service('/soma_llsd/update_scene',UpdateScene,self.update_scene_cb)
        #insert_scene_auto - TODO


        get_segment = rospy.Service('/soma_llsd/get_segment',GetSegment,self.get_segment_cb)
        insert_segment = rospy.Service('/soma_llsd/insert_segment',InsertSegment,self.insert_segment_cb)
        add_observations_to_segment = rospy.Service('/soma_llsd/add_observations_to_segment',AddObservationsToSegment,self.add_obs_cb)


        rospy.spin()

    def add_obs_cb(self,req):
        rospy.loginfo("LLSD: -- Request to add observations to segment")
        b = self.add_observations_to_segment(req.segment_id,req.observations,req.scene_id)
        result = AddObservationsToSegmentResponse(b)
        return result

    def insert_segment_cb(self,req):
        rospy.loginfo("LLSD: -- Request to insert segment recieved")
        b,r = self.insert_segment(req.meta_data,req.scene_id,req.observations)
        result = InsertSegmentResponse(b,r)
        return result

    def get_segment_cb(self,req):
        r,s = self.get_segment(req.segment_id)
        result = GetSegmentResponse(r,s)
        return result

    def get_scene_cb(self,req):
        r,s = self.get_scene(req.scene_id)
        result = GetSceneResponse(r,s)
        return result

    def update_scene_cb(self,req):
        rospy.loginfo("LLSD: -- Request to update scene recieved")
        sc = self.update_scene(req.input)
        result = UpdateSceneResponse(sc)
        return result

    def update_segment_cb(self,req):
        rospy.loginfo("LLSD: -- Request to update segment recieved")
        sc = self.update_segment(req.input)
        result = UpdateSegmentResponse(sc)
        return result

    def insert_scene_cb(self,req):
        rospy.loginfo("LLSD: -- Request to insert scene recieved")
        b,r = self.insert_scene(req.episode_id,
        req.waypoint,
        req.meta_data,
        req.timestamp,
        req.transform,
        req.cloud,
        req.rgb_img,
        req.depth_img,
        req.camera_info,
        req.robot_pose)
        result = InsertSceneResponse(b,r)
        return result

    def insert_scene_auto_cb(self,req):
        rospy.loginfo("LLSD: -- Request to auto insert scene recieved")
        b,r = self.insert_scene_auto(req)
        result = InsertSceneAutoResponse(b,r)
        return result


    def insert_scene_auto(self,req):
        # get the messages from topics
        listener = TransformationStore()
        listener.create_live()
        print("waiting for listener")
        rospy.sleep(2)
        listener.kill()
        generated_transform = listener.get_as_msg()

        generated_camera_info = rospy.wait_for_message(req.camera_info_topic, CameraInfo, timeout=10.0)
        generated_robot_pose = rospy.wait_for_message(req.robot_pose_topic, geometry_msgs.msg.Pose, timeout=10.0)

        sc = self.insert_scene(req.episode_id,
        req.waypoint,
        req.meta_data,
        req.timestamp,
        generated_transform,
        req.cloud,
        req.rgb_img,
        req.depth_img,
        generated_camera_info,
        generated_robot_pose)

        return sc

    def insert_scene(self,episode_id,waypoint,meta_data,timestamp,tf,cloud,rgb_img,depth_img,camera_info,robot_pose):
        try:
            new_scene = Scene()
            new_scene.id = str(uuid.uuid4())
            new_scene.episode_id = episode_id
            new_scene.waypoint = waypoint
            new_scene.meta_data = meta_data
            new_scene.timestamp = timestamp
            new_scene.transform = tf
            new_scene.cloud = cloud
            new_scene.rgb_img = rgb_img
            new_scene.depth_img = depth_img
            new_scene.camera_info = camera_info
            new_scene.robot_pose = robot_pose
            self.scene_store.insert_named(new_scene.id,new_scene)
            rospy.loginfo("LLSD: -- Scene successfully inserted with id " + new_scene.id)
            return True,new_scene
        except Exception,e:
            rospy.loginfo(e)
            return False,Scene()

    def get_scene(self,scene_id):
        scene,meta = self.scene_store.query_named(scene_id, Scene._type)
        if(not scene):
            print("Unable to find scene with ID: " + scene_id)
            return False,None
        return True,scene

    def update_scene(self,scene):
        try:
            self.scene_store.update_named(scene.id, scene)
            rospy.loginfo("LLSD: -- Scene successfully updated")
            return True
        except Exception,e:
            rospy.loginfo(e)
            return False

    def update_segment(self,segment):
        try:
            self.segment_store.update_named(segment.id, segment)
            rospy.loginfo("LLSD: -- Segment successfully updated")
            return True
        except Exception,e:
            rospy.loginfo(e)
            return False

    def insert_segment(self,meta_data,scene_id,observations):
        try:
            new_segment = Segment()
            new_segment.id = str(uuid.uuid4())
            #new_segment.timestamp = scene.timestamp
            new_segment.meta_data = meta_data
            #new_segment.scene_id = scene_id
            new_segment.observations = observations
            new_segment.related_scenes = [scene_id]
            self.segment_store.insert_named(new_segment.id,new_segment)
            rospy.loginfo("LLSD: -- Success! added segment " + new_segment.id)
            return True,new_segment
        except Exception,e:
            rospy.loginfo(e)
            return False,Segment()

    def get_segment(self,segment_id):
        rospy.loginfo("LLSD: received get segment for: " + segment_id)
        segment,meta = self.segment_store.query_named(segment_id, Segment._type)
        if(not segment):
            print("Unable to find segment with ID: " + segment_id)
            return False,None
        rospy.loginfo("LLSD: success, returning segment!")
        return True,segment

    def update_segment(self,segment):
        try:
            self.segment_store.update_named(segment.id,segment)
            return True
        except Exception,e:
            rospy.loginfo(e)
            return False

    def add_observations_to_segment(self,segment_id,observations,scene_id):
        try:
            result,segment = self.get_segment(segment_id)

            if(result is False):
                rospy.loginfo("LLSD: Unable to find segment with ID: " + segment_id)
                return False

            if(scene_id not in segment.related_scenes):
                segment.related_scenes.append(scene_id)

            for o in observations:
                o.scene_id = scene_id
                if(o.id is None):
                    o.id = str(uuid.uuid4())
                segment.observations.append(o)

            self.segment_store.update_named(segment_id,segment)
            rospy.loginfo("LLSD: -- Observations successfully added to segment")
            return True
        except Exception,e:
            rospy.loginfo(e)
            return False

class TransformationStore():
    """
    Subscribes to /TF, stores transforms, turns into transformer when needed
    """
    def __init__(self):
        self._transformations = deque([])
        self._lively = False
        self._max_buffer = 10

    def kill(self):
        self._sub.unregister()

    def cb(self, transforms):
        time_window = rospy.Duration(self._max_buffer)
        for transform in transforms.transforms:
            #rospy.loginfo("LLSD: Got transform: %s - > %s"% ( transform.header.frame_id, transform.child_frame_id))
            if self._max_buffer > 0 and len(self._transformations) > 2:
                l =  self._transformations.popleft()
                if (transform.header.stamp -  l.header.stamp) < time_window:
                    self._transformations.appendleft(l)
            self._transformations.append(transform)

    def create_from_transforms(self, transforms):
        """
        Create a store from a given set of transforms
        transforms: Must be a list of TransformStamped messages
        """
        for t in transforms:
            self._transformations.append(t)
        return self

    def create_live(self, max_buffer=2.0):
        # subscribe to tf and store transforms
        self._max_buffer = max_buffer
        self._sub = rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, self.cb)
        self._lively = True
        return self

    def get_as_msg(self):
        msg = tf2_msgs.msg.TFMessage()
        msg.transforms = self._transformations
        return msg

    def msg_to_transformer(self,msg):
        t = tf.TransformerROS()
        for transform in msg.transforms:
            t.setTransform(transform)
        return t

if __name__ == '__main__':

    parser = argparse.ArgumentParser(prog='server.py')
    parser.add_argument("--db_name", nargs='?', help='Name of the database')
    parser.add_argument('--llsd_segment_store_name', nargs='?', help='Name of the collection used to store low-level segments')
    parser.add_argument('--llsd_scene_store_name', nargs='?', help='Name of the collection used to store low-level scenes')

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    if args.db_name is not None:
        if args.llsd_scene_store_name is not None:
            StoreController(args.db_name,args.llsd_scene_store_name,args.llsd_segment_store_name)
        else:
            StoreController(args.db_name)
    else:
        rospy.loginfo("LLSD: Running SOMA LLSD manager with defaults")
        StoreController()
