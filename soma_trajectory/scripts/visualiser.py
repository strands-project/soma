#!/usr/bin/env python

import rospy
from soma_msgs.msg import SOMAROIObject
from soma_msgs.msg import SOMATrajectory
from soma_map_manager.srv import MapInfo
from soma_manager.srv import SOMAQueryROIs
from mongodb_store.message_store import MessageStoreProxy
from soma_trajectory.srv import SOMAQueryTrajectories, SOMAQueryTrajectoriesRequest

import sys
import datetime
import numpy as np
from Tkinter import *


def average_velocity(traj):
    start = traj.start_time
    end = traj.end_time
    delta = float((end-start).secs + 0.000000001 * (end-start).nsecs)
    avg_vel = 0.0
    if delta != 0.0:
        avg_vel = traj.length / delta
    return avg_vel


class TrajectoryQueryVisualiser(object):

    def __init__(self):
        # map
        resp = self._init_map()
        if resp is None:
            rospy.logerr("No map info received. Quitting SOMA ROI manager...")
            return
        self.soma_map_name = resp.map_name
        self.map_unique_id = resp.map_unique_id
        str_msg = "Map name: ", self.soma_map_name, " Unique ID: ", self.map_unique_id
        rospy.loginfo(str_msg)
        # roi and trajectory service
        # self.roi_query_srv = self._get_soma_roi_queryservice()
        self.rois = self._get_regions_and_configs()
        self.traj_query_srv = self._get_soma_trajectory_queryservice()
        self.start_time, self.end_time = self._get_trajectory_time_interval()
        # interface
        self._selected_config = ""
        self._main_window = Tk()
        self._main_window.wm_title("SOMA Trajectory")
        self._create_upper_frame()
        rospy.Timer(rospy.Duration(1), self._get_info_from_window)
        self._main_window.mainloop()

    def _get_info_from_window(self, event):
        try:
            _selected_config = self.config_list.get(
                self.config_list.curselection()[0]
            )
            if _selected_config != self._selected_config:
                if self._selected_config != "":
                    self.region_list.delete(
                        0, len(self.rois[self._selected_config])-1
                    )
                for idx, roi in enumerate(self.rois[_selected_config]):
                    self.region_list.insert(idx, roi)
                self._selected_config = _selected_config
        except IndexError:
            _selected_config = ""
        weekday_vars = [i.get() for i in self.weekday_vars]
        if 1 in weekday_vars:
            self.low_bound_scale.config(state=DISABLED)
            self.upper_bound_scale.config(state=DISABLED)
        else:
            self.low_bound_scale.config(state=NORMAL)
            self.upper_bound_scale.config(state=NORMAL)

    def _create_upper_frame(self):
        start_time = (self.start_time.secs / 3600) * 3600
        end_time = (self.end_time.secs / 3600) * 3600 + 3600
        delta_time = (end_time - start_time) / 3600
        # outer upper frame for temporal constraints
        upper_frame = LabelFrame(
            self._main_window, text="SOMA Trajectory Queries",
            padx=5, pady=5
        )
        upper_frame.pack(side=TOP)
        # lower bound slider
        Label(
            upper_frame, text="Lower Time Bound (in hour)"
        ).pack(anchor=W)
        self.low_bound_time = IntVar()
        self.low_bound_scale = Scale(
            upper_frame, variable=self.low_bound_time, orient=HORIZONTAL,
            from_=0, to=delta_time, length=550
        )
        self.low_bound_scale.pack()
        label_lower_bound = Frame(upper_frame)
        label_lower_bound.pack(side=TOP, fill=X)
        Label(
            label_lower_bound, text=datetime.datetime.fromtimestamp(
                (self.start_time.secs / 3600) * 3600
            )
        ).pack(side=LEFT)
        Label(
            label_lower_bound, text=datetime.datetime.fromtimestamp(
                (self.end_time.secs / 3600) * 3600 + 3600
            )
        ).pack(side=RIGHT)
        Label(upper_frame, text=" ").pack()
        Label(
            upper_frame, text="Upper Time Bound (in hour, must be greater than lower bound)"
        ).pack(anchor=W)
        self.upper_bound_time = IntVar()
        self.upper_bound_scale = Scale(
            upper_frame, variable=self.upper_bound_time, orient=HORIZONTAL,
            from_=0, to=delta_time, length=550
        )
        self.upper_bound_scale.pack()
        label_upper_bound = Frame(upper_frame)
        label_upper_bound.pack(side=TOP, fill=X)
        Label(
            label_upper_bound, text=datetime.datetime.fromtimestamp(
                (self.start_time.secs / 3600) * 3600
            )
        ).pack(side=LEFT)
        Label(
            label_upper_bound, text=datetime.datetime.fromtimestamp(
                (self.end_time.secs / 3600) * 3600 + 3600
            )
        ).pack(side=RIGHT)
        # inner frame for special temporal constraints (weekdays, hour)
        Label(upper_frame, text=" ").pack()
        tempo_upper_frame = LabelFrame(
            upper_frame, text="Periodic Temporal Query",
            padx=3, pady=3
        )
        tempo_upper_frame.pack(side=TOP, fill=X)
        week_frame = Frame(tempo_upper_frame)
        week_frame.pack()
        weekdays = {
            0: "Monday", 1: "Tuesday", 2: "Wednesday", 3: "Thursday",
            4: "Friday", 5: "Saturday", 6: "Sunday"
        }
        self.weekday_vars = list()
        for idx in range(7):
            var = IntVar()
            Checkbutton(
                week_frame, text=weekdays[idx], variable=var,
            ).pack(side=LEFT, anchor=W, expand=YES)
            self.weekday_vars.append(var)
        Label(tempo_upper_frame, text=" ").pack(side=TOP)
        hour_label_frame = Frame(tempo_upper_frame, padx=10)
        hour_label_frame.pack(fill=X)
        Label(
            hour_label_frame, text="Lower Hour Bound"
        ).pack(side=LEFT, anchor=W)
        Label(
            hour_label_frame, text="Upper Hour Bound                               "
        ).pack(side=RIGHT, anchor=W)
        self.lower_hour = IntVar()
        self.upper_hour = IntVar()
        hour_frame = Frame(tempo_upper_frame, padx=10)
        hour_frame.pack(fill=X)
        Scale(
            hour_frame, variable=self.lower_hour, orient=HORIZONTAL,
            from_=0, to=24, length=240
        ).pack(side=LEFT)
        Scale(
            hour_frame, variable=self.upper_hour, orient=HORIZONTAL,
            from_=0, to=24, length=240
        ).pack(side=RIGHT)
        # iner upper frame for spatial constraints
        Label(upper_frame, text=" ").pack()
        inner_upper_frame = LabelFrame(
            upper_frame, text="Regions of Interest",
            padx=3, pady=3
        )
        inner_upper_frame.pack(side=TOP, fill=X)
        label_inner_frame = Frame(inner_upper_frame)
        label_inner_frame.pack(side=TOP, fill=X)
        Label(label_inner_frame, text="Configurations").pack(side=LEFT)
        Label(
            label_inner_frame,
            text="Regions                                                      "
        ).pack(side=RIGHT)
        listbox_frame = Frame(inner_upper_frame)
        listbox_frame.pack(side=TOP, fill=X)
        self.config_list = Listbox(
            listbox_frame, selectmode=SINGLE, fg="black",
            bg="white", width=33, height=5
        )
        for idx, config in enumerate(self.rois.keys()):
            self.config_list.insert(idx, config)
        self.config_list.pack(side=LEFT)
        self.region_list = Listbox(
            listbox_frame, selectmode=SINGLE, fg="black",
            bg="white", width=33, height=5
        )
        self.region_list.pack(side=RIGHT)
        Button(upper_frame, text="Query", command=self._query).pack(anchor=CENTER, fill=X)

        Label(upper_frame, text=" ").pack()
        lower_frame = LabelFrame(
            upper_frame, text="Info Terminal", labelanchor=N, padx=5, pady=5, height=100
        )
        lower_frame.pack(side=TOP, fill=X)
        self.message = StringVar()
        Message(
            lower_frame, textvariable=self.message,  justify=LEFT, padx=3, pady=3,
            bg="white", fg="black", width=500
        ).pack(anchor=CENTER, fill=BOTH)

    def _query(self):
        # temporal
        lower_hour = self.lower_hour.get()
        upper_hour = self.upper_hour.get()
        weekdays = [i.get() for i in self.weekday_vars]
        useweekday = 1 in weekdays
        low_bound_timestamp = (self.start_time.secs / 3600) * 3600 + self.low_bound_time.get()*3600
        low_bound_timestamp = rospy.Time(low_bound_timestamp)
        upp_bound_timestamp = (self.start_time.secs / 3600) * 3600 + self.upper_bound_time.get()*3600
        upp_bound_timestamp = rospy.Time(upp_bound_timestamp)
        if not useweekday and upp_bound_timestamp <= low_bound_timestamp:
            msg = "Lower threshold can not be higher than upper threshold!"
            msg += "\nCurrent query is ignored..."
            self.message.set(msg)
            return
        if useweekday and upper_hour <= lower_hour:
            msg = "Lower threshold can not be higher than upper threshold!"
            msg += "\nCurrent query is ignored..."
            self.message.set(msg)
            return
        # spatial
        region = ""
        config = self._selected_config
        if config != "":
            try:
                region = self.region_list.get(self.region_list.curselection()[0])
            except IndexError:
                config = ""
                region = ""
        # info preprocess
        date_low_bound = datetime.datetime.fromtimestamp(low_bound_timestamp.secs)
        date_upp_bound = datetime.datetime.fromtimestamp(upp_bound_timestamp.secs)
        msg = "Requesting visualisation between %s to %s" % (date_low_bound, date_upp_bound)
        if region != "":
            msg += " for region %s with config %s..." % (region, config)
        self.message.set(msg)
        # calling service
        try:
            res = self.traj_query_srv(
                SOMAQueryTrajectoriesRequest(
                    useweekday=useweekday, usehourtime=useweekday,
                    start_time=low_bound_timestamp, end_time=upp_bound_timestamp,
                    weekdays=weekdays, lowerhour=lower_hour, upperhour=upper_hour,
                    region_id=region, region_config=config, visualize=True
                )
            )
        except rospy.service.ServiceException:
            rospy.logerr("Pymongo on mongodb_store is having timeout...")
            rospy.logerr("The query is iqnored...")
            self.message.set("Pymongo on mongodb_store is having timeout, your query is ignored.")
            return
        self.trajectory_analysis(
            region, config, res.trajectories, low_bound_timestamp,
            upp_bound_timestamp, lower_hour, upper_hour, weekdays
        )

    def trajectory_analysis(
        self, region, config, trajectories, low_bound_timestamp,
        upp_bound_timestamp, lower_hour, upper_hour, weekdays
    ):
        if region != "":
            msg = "Trajectory Analysis on Region %s\n\n" % region
        else:
            msg = "General Trajectory Analysis\n\n"
        msg += "Total trajectories: %d\n" % len(trajectories)
        if 1 not in weekdays:
            msg += "Average trajectories per hour: %.2f\n" % (
                len(trajectories) / ((upp_bound_timestamp - low_bound_timestamp).secs / 3600.0)
            )
        else:
            try:
                last_weekday = trajectories[-1].logday
                first_time = (trajectories[0].logtimestamp / 86400) * 86400
                last_time = (((trajectories[-1].logtimestamp / 86400) + 1) * 86400) - 1
                number_of_weeks = (last_time - first_time) / 604800
                residue_days = ((last_time - first_time) % 604800) / 86400
                if ((last_time - first_time) % 604800) % 86400 != 0:
                    residue_days += 1
                residue_days = range(last_weekday-residue_days+1, last_weekday+1)
                residue_days = [
                    int(idx-7 in residue_days or idx in residue_days) for idx in range(7)
                ]
                avg_trajectories = len(trajectories) / (
                    ((number_of_weeks * sum(weekdays)) + (
                        sum(np.array(residue_days) * np.array(weekdays))
                    )) * float(upper_hour - lower_hour)
                )
            except IndexError:
                avg_trajectories = 0.0
            msg += "Average trajectories per hour: %.2f\n" % avg_trajectories
        trajectory_lengths = [i.length for i in trajectories]
        try:
            msg += "Average trajectory length: %.2f, with standard deviation: %.2f" % (
                sum(trajectory_lengths) / float(len(trajectories)), np.std(trajectory_lengths)
            )
        except ZeroDivisionError:
            msg += "Average trajectory length: 0.0, with standard deviation: 0.0"
        self.message.set(msg)

    def _get_trajectory_time_interval(self):
        start_time = None
        end_time = None
        msg_store = MessageStoreProxy(database="somadata", collection="trajectory")
        try:
            logs = msg_store.query(
                SOMATrajectory._type, message_query={
                    "map_name": self.soma_map_name,
                    "map_unique_id": self.map_unique_id
                }, sort_query=[("logtimestamp", 1)],
                projection_query={"logtimestamp": 1}, single=True
            )
            start_time = rospy.Time(logs[0].logtimestamp)
            logs = msg_store.query(
                SOMATrajectory._type, message_query={
                    "map_name": self.soma_map_name,
                    "map_unique_id": self.map_unique_id
                }, sort_query=[("logtimestamp", -1)],
                projection_query={"logtimestamp": 1}, single=True
            )
            end_time = rospy.Time(logs[0].logtimestamp)
        except rospy.service.ServiceException:
            rospy.loginfo("There is no trajectory collection in somadata database")
            sys.exit(2)
        return start_time, end_time

    def _get_regions_and_configs(self):
        configs = dict()
        msg_store = MessageStoreProxy(database="somadata", collection="roi")
        try:
            logs = msg_store.query(
                SOMAROIObject._type, message_query={
                    "map_name": self.soma_map_name,
                    "map_unique_id": self.map_unique_id
                }, projection_query={"id":1, "config":1},
                sort_query=[("logtimestamp", -1)]
            )
            for log in logs:
                if log[0].config not in configs:
                    configs[log[0].config] = list()
                if log[0].id not in configs[log[0].config]:
                    configs[log[0].config].append(log[0].id)
        except rospy.service.ServiceException:
            rospy.loginfo("There is no roi collection in somadata database")
            sys.exit(2)
        return configs

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

    def _get_soma_trajectory_queryservice(self):
        rospy.loginfo("Waiting for the SOMA trajectory query service...")
        try:
            rospy.wait_for_service("soma/query_trajectories", timeout=30)
            return rospy.ServiceProxy(
                "soma/query_trajectories", SOMAQueryTrajectories
            )
        except rospy.ROSException, e:
            rospy.logwarn("Service call failed: %s" % e)
            rospy.loginfo("Terminating all...")
            sys.exit(2)


if __name__ == '__main__':
    rospy.init_node("soma_trajectory_visualiser")
    TrajectoryQueryVisualiser()
