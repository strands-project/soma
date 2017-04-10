#!/usr/bin/env python

import rospy
import argparse
from human_trajectory.msg import Trajectories
from human_trajectory.trajectories import OfflineTrajectories
from soma_trajectory.soma_trajectory_manager import SOMATrajectoryManager


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog='trajectory_importer.py')
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
    trajs = OfflineTrajectories()
    soma_trajs = SOMATrajectoryManager(
        db_name=args.db_name, collection_name=args.collection_name
    )
    for uuid, traj in trajs.traj.iteritems():
        soma_trajs.add_trajectory(traj.get_trajectory_message())
