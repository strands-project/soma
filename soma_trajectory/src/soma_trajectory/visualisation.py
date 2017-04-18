#!/usr/bin/env python

import rospy
from threading import Lock
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import interactive_markers.interactive_marker_server as ims
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarker


class TrajectoryVisualisation(object):

    def __init__(self, marker_name):
        self._lock = Lock()
        self._modulo = 3
        self._visualised_ids = list()
        self._server = ims.InteractiveMarkerServer(marker_name)
        rospy.Timer(rospy.Duration(5), self._remove_trajectories)

    def _remove_trajectories(self, event):
        now = rospy.Time.now()
        remove_ids = list()
        self._lock.acquire()
        for idx in range(len(self._visualised_ids)):
            if now - self._visualised_ids[idx][1] > rospy.Duration(60):
                self._server.erase(self._visualised_ids[idx][0])
                self._server.applyChanges()
                remove_ids.append(self._visualised_ids[idx][0])
        # rospy.loginfo("Removing %d trajectories from visualisation..." % len(remove_ids))
        self._visualised_ids = [
            i for i in self._visualised_ids if i[0] not in remove_ids
        ]
        self._lock.release()

    def delete_trajectories(self):
        self._lock.acquire()
        for idx in range(len(self._visualised_ids)):
            self._server.erase(self._visualised_ids[idx][0])
            self._server.applyChanges()
        # rospy.loginfo("Removing %d trajectories from visualisation..." % len(remove_ids))
        self._visualised_ids = list()
        self._lock.release()

    def visualize_trajectories(self, trajectories):
        for trajectory in trajectories:
            self._visualised_ids.append((trajectory.id, rospy.Time.now()))
            int_marker = self.create_trajectory_marker(trajectory)
            self._server.insert(int_marker, self._update_cb)
            self._server.applyChanges()

    def _update_cb(self, feedback):
        return

    def set_line_color(self, **args):
        color = ColorRGBA()
        color.r = 0.0
        color.g = float(args["index"]) / float(args["length"])
        color.b = 1.0 - float(args["index"]) / float(args["length"])
        color.a = 1.0
        return color

    def create_trajectory_marker(self, traj):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/map"
        int_marker.name = traj.id
        int_marker.description = traj.id

        int_marker.pose = traj.posearray.poses[0]
        int_marker.pose.orientation.x = 0
        int_marker.pose.orientation.y = 0
        int_marker.pose.orientation.z = 0
        int_marker.pose.orientation.w = 1

        line_marker = Marker()
        line_marker.type = Marker.LINE_STRIP
        line_marker.scale.x = 0.05

        line_marker.points = []
        while len(traj.posearray.poses) / self._modulo > 5000:
            self._modulo += 1

        for i, pose in enumerate(traj.posearray.poses):
            if i % self._modulo == 0:
                p = Point(
                    pose.position.x - int_marker.pose.position.x,
                    pose.position.y - int_marker.pose.position.y,
                    0.0
                )
                line_marker.points.append(p)
                line_marker.colors.append(
                    self.set_line_color(index=i, length=len(traj.posearray.poses))
                )

        self._modulo = 3

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.markers.append(line_marker)
        int_marker.controls.append(control)

        return int_marker
