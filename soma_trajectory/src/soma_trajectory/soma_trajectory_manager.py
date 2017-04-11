#!/usr/bin/env python

import math
import copy
import rospy
import argparse
from threading import Lock
from datetime import datetime
from geometry_msgs.msg import PoseArray
from soma_map_manager.srv import MapInfo
from soma_msgs.msg import SOMATrajectory
from human_trajectory.msg import Trajectories
from mongodb_store.message_store import MessageStoreProxy
from soma_trajectory.visualisation import TrajectoryVisualisation
from soma_trajectory.srv import SOMAQueryTrajectories, SOMAQueryTrajectoriesResponse


def coords_to_lnglat(x, y):
        earth_radius = 6371000.0    # in meters
        lng = 90 - math.degrees(math.acos(float(x) / earth_radius))
        lat = 90 - math.degrees(math.acos(float(y) / earth_radius))
        return [lng, lat]


class SOMATrajectoryManager(object):

    def __init__(self, db_name="somadata", collection_name="trajectory"):
        rospy.loginfo("Initiating soma trajectory manager...")
        # trajectory stuff
        self._lock = Lock()
        self._trajectories = list()
        self._soma_traj_ids = dict()
        # trajectory visualisation stuff
        self._visualisation = TrajectoryVisualisation(rospy.get_name())
        # db stuff
        self.db_name = db_name
        self.collection_name = collection_name
        self._msg_store = MessageStoreProxy(database=self.db_name, collection=self.collection_name)
        # Get the SOMA map name and unique id
        resp = self._init_map()
        if resp is None:
            rospy.logerr("No map info received. Quitting SOMA ROI manager...")
            return
        self.soma_map_name = resp.map_name
        self.map_unique_id = resp.map_unique_id
        str_msg = "Map name: ", self.soma_map_name, " Unique ID: ", self.map_unique_id
        rospy.loginfo(str_msg)
        # service
        rospy.loginfo("Initiating soma/query_trajectories service...")
        rospy.Service(
            'soma/query_trajectories', SOMAQueryTrajectories, self._srv_cb
        )
        rospy.sleep(1)
        rospy.loginfo("Service is ready...")

    def _srv_cb(self, srv):
        rospy.loginfo(
            "Got a request to display trajectories (limited to 10000) within %d and %d..."
            % (srv.start_time.secs, srv.end_time.secs)
        )
        query = {
            "map_name": self.soma_map_name,
            "map_unique_id": self.map_unique_id
        }
        if srv.start_time.secs != 0:
            query.update({"start_time.secs": {"$gte": srv.start_time.secs}})
        if srv.end_time.secs != 0:
            query.update({"end_time.secs": {"$lt": srv.end_time.secs}})
        logs = self._msg_store.query(
            SOMATrajectory._type, message_query=query, limit=10000
        )
        trajectories = [log[0] for log in logs]
        if srv.visualize:
            rospy.loginfo("Visualising %d trajectories..." % len(trajectories))
            self._visualisation.visualize_trajectories(trajectories)
        return SOMAQueryTrajectoriesResponse(trajectories)

    def online_conversion(self, topic="/people_trajectory/trajectories/complete"):
        rospy.loginfo(
            "Initiating online trajectory conversion from %s topic to SOMA format..." % topic
        )
        sub = rospy.Subscriber(topic, Trajectories, self._traj_cb, None, 10)
        rospy.sleep(1)
        while not rospy.is_shutdown():
            if len(self._trajectories):
                soma_trajectories = list()
                trajectories = copy.deepcopy(self._trajectories)
                for trajectory in trajectories:
                    traj = self.add_trajectory(trajectory)
                    soma_trajectories.append(traj)
                self._visualisation.visualize_trajectories(soma_trajectories)
                rospy.loginfo("%d trajectories are converted..." % len(trajectories))
                self._lock.acquire()
                self._trajectories = [
                    i for i in self._trajectories if i not in trajectories
                ]
                self._lock.release()
            rospy.sleep(1)
        rospy.loginfo("Shutting down...")
        sub.unregister()

    def _traj_cb(self, trajectories):
        self._lock.acquire()
        try:
            self._trajectories.extend(trajectories.trajectories)
        finally:
            self._lock.release()
        rospy.sleep(0.1)

    def _init_map(self):
        rospy.loginfo("SOMA trajectory manager is waiting for the SOMA map info...")
        try:
            rospy.wait_for_service('soma/map_info', timeout=30)
        except:
            return None
        try:
            map_info = rospy.ServiceProxy('soma/map_info', MapInfo)
            resp1 = map_info(0)
            return resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return None

    def add_trajectory(self, trajectory):
        soma_trajectory = SOMATrajectory()

        soma_trajectory.header.frame_id = '/map'
        soma_trajectory.header.stamp = rospy.get_rostime()
        soma_trajectory.id = trajectory.uuid
        soma_trajectory.map_name = self.soma_map_name
        soma_trajectory.map_unique_id = self.map_unique_id
        self.insert_soma_time_fields(soma_trajectory, trajectory.start_time.secs)
        soma_trajectory.posearray.header = soma_trajectory.header
        soma_trajectory.posearray.poses = [
            i.pose for i in trajectory.trajectory
        ]
        soma_trajectory.geotype = 'LineString'
        self.insert_geo_poses(soma_trajectory)
        soma_trajectory.length = trajectory.trajectory_length
        soma_trajectory.start_time = trajectory.start_time
        soma_trajectory.end_time = trajectory.end_time
        _id = self._msg_store.insert(soma_trajectory)
        self._soma_traj_ids[soma_trajectory.id] = _id
        return soma_trajectory

    def insert_soma_time_fields(self, soma_obj, timestamp=None):
        if timestamp is None:
            # convert to uint32
            soma_obj.logtimestamp = int(rospy.get_rostime().to_sec()) & 0xffffffff
            dt = datetime.utcfromtimestamp(soma_obj.logtimestamp)
            soma_obj.loghour = int(dt.hour)
            soma_obj.logminute = int(dt.minute)
            soma_obj.logday = int(dt.weekday())+1
            soma_obj.logtimeminutes = int(dt.hour)*60 + int(dt.minute)
        else:
            # convert to uint32
            soma_obj.logtimestamp = timestamp & 0xffffffff
            dt = datetime.utcfromtimestamp(soma_obj.logtimestamp)
            soma_obj.loghour = int(dt.hour)
            soma_obj.logminute = int(dt.minute)
            soma_obj.logday = int(dt.weekday())
            soma_obj.logtimeminutes = int(dt.hour)*60 + int(dt.minute)

    def insert_geo_poses(self, msg):
        coordinates = PoseArray()
        for pose in msg.posearray.poses:
            p = copy.deepcopy(pose)
            p.orientation.w = 0.0
            res = coords_to_lnglat(p.position.x, p.position.y)
            p.position.x = res[0]
            p.position.y = res[1]
            coordinates.poses.append(p)
        msg.geoposearray = coordinates


if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog='soma_trajectory_manager.py')
    parser.add_argument(
        '--db_name',
        help='Name of the soma trajectory db (default = somadata)',
        default="somadata"
    )
    parser.add_argument(
        '--collection_name',
        help='Name of the soma trajectory collection (default = trajectory)',
        default="trajectory"
    )
    args = parser.parse_args()

    rospy.init_node("soma_trajectory")
    soma_trajectory = SOMATrajectoryManager(
        db_name=args.db_name, collection_name=args.collection_name
    )
    soma_trajectory.online_conversion()
    rospy.spin()
