#!/usr/bin/env python

from geometry_msgs.msg import PoseArray
from soma_map_manager.srv import MapInfo
from soma_msgs.msg import SOMATrajectory
from soma_manager.srv import SOMAQueryROIs
from human_trajectory.msg import Trajectories
from mongodb_store.message_store import MessageStoreProxy
from soma_trajectory.visualisation import TrajectoryVisualisation
from soma_trajectory.srv import SOMAQueryTrajectories, SOMAQueryTrajectoriesResponse

import math
import copy
import rospy
import pymongo
import argparse
import itertools
import numpy as np
from threading import Lock
from datetime import datetime
from shapely.geometry import Polygon, LineString


def create_line_string(points):
    return LineString(points)


def create_polygon(posearray):
    xs = [i.position.x for i in posearray.poses]
    ys = [i.position.y for i in posearray.poses]
    if Polygon(np.array(zip(xs, ys))).area == 0.0:
        xs = [
            [xs[0]] + list(i) for i in itertools.permutations(xs[1:])
        ]
        ys = [
            [ys[0]] + list(i) for i in itertools.permutations(ys[1:])
        ]
        areas = list()
        for ind in range(len(xs)):
            areas.append(Polygon(np.array(zip(xs[ind], ys[ind]))))
        return Polygon(
            np.array(zip(xs[areas.index(max(areas))], ys[areas.index(max(areas))]))
        )
    else:
        return Polygon(np.array(zip(xs, ys)))


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
        self.roi_query_srv = self._get_soma_roi_queryservice()
        rospy.loginfo("Initiating soma/query_trajectories service...")
        rospy.Service(
            'soma/query_trajectories', SOMAQueryTrajectories, self._srv_cb
        )
        rospy.sleep(1)
        rospy.loginfo("Service is ready...")

    def _get_soma_roi_queryservice(self):
        rospy.loginfo("Waiting for the SOMA query ROI service...")
        try:
            rospy.wait_for_service('soma/query_rois', timeout=30)
            return rospy.ServiceProxy('soma/query_rois', SOMAQueryROIs)
        except rospy.ROSException, e:
            rospy.logwarn("Service call failed: %s" % e)
            rospy.logwarn("Specific region queries will be ignored...")
            return None

    def _create_index(self):
        # create index for the collection
        db = pymongo.MongoClient(
            rospy.get_param("mongodb_host", "localhost"),
            rospy.get_param("mongodb_port", 62345)
        ).get_database(self.db_name)
        if self.collection_name in db.collection_names():
            collection = db.get_collection(self.collection_name)
            total = collection.count()
            if total == 1 or total % 10 == 0:
                collection.create_index([("logtimestamp", pymongo.ASCENDING)])

    def _get_temporal_query(self, srv):
        query = {
            "map_name": self.soma_map_name,
            "map_unique_id": self.map_unique_id,
        }
        if not srv.useweekday and not srv.usehourtime:
            logtimestamp_query = {"$gte": 0, "$lt": rospy.Time.now().secs}
            if srv.start_time.secs != 0:
                logtimestamp_query["$gte"] = srv.start_time.secs
            if srv.end_time.secs != 0:
                logtimestamp_query["$lt"] = srv.end_time.secs
            query.update({"logtimestamp": logtimestamp_query})
            rospy.loginfo(
                "Got a request to query trajectories (limited to 10000) within %d and %d..."
                % (logtimestamp_query["$gte"], logtimestamp_query["$lt"])
            )
        else:
            msg = "Got a request to query trajectories "
            if srv.useweekday:
                weekdays = [i for i in range(7) if srv.weekdays[i]]
                query.update({"logday": {"$in": weekdays}})
                msg += "for weekday %s " % str(weekdays)
            if srv.usehourtime:
                loghour_query = {"$gte": 0, "$lt": 24}
                if srv.lowerhour != 0:
                    loghour_query["$gte"] = srv.lowerhour
                if srv.upperhour != 0:
                    loghour_query["$lt"] = srv.upperhour
                query.update({"loghour": loghour_query})
                msg += "within hour %d and %d..." % (
                    loghour_query["$gte"], loghour_query["$lt"]
                )
            rospy.loginfo(msg)
        return query

    def _get_spatial_query(self, srv):
        region = None
        if self.roi_query_srv is not None and srv.region_config != "" and srv.region_id != "":
            spatial_query = dict()
            response = self.roi_query_srv(
                query_type=0, roiconfigs=[srv.region_config],
                roiids=[srv.region_id], returnmostrecent=True
            )
            for reg in response.rois:
                if reg.id == srv.region_id and reg.config == srv.region_config:
                    region = reg
            if region is not None:
                region = create_polygon(region.posearray)
            else:
                rospy.logwarn("Specified region does not exist, ignore...")
        return region

    def _srv_cb(self, srv):
        # spatial query specification
        region = self._get_spatial_query(srv)
        # temporal query specification
        query =  self._get_temporal_query(srv)
        logs = self._msg_store.query(
            SOMATrajectory._type, message_query=query, limit=10000,
            sort_query=[("logtimestamp", 1)]
        )
        # product of temporal and spatial query
        trajectories = list()
        for log in logs:
            if region is not None:
                line_string = create_line_string(
                    [
                        (
                            i.position.x, i.position.y
                        ) for h, i in enumerate(log[0].posearray.poses) if h % 3 == 0
                    ]
                )
                if not region.intersects(line_string):
                    continue
            trajectories.append(log[0])
        if srv.visualize:
            self._visualisation.delete_trajectories()
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
                _ids = [traj.uuid for traj in trajectories]
                for trajectory in trajectories:
                    traj = self.add_trajectory(trajectory)
                    soma_trajectories.append(traj)
                self._visualisation.visualize_trajectories(soma_trajectories)
                rospy.loginfo("%d trajectories are converted..." % len(trajectories))
                self._lock.acquire()
                self._trajectories = [
                    i for i in self._trajectories if i.uuid not in _ids
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
        self._create_index()
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
