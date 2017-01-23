#!/usr/bin/env python

import rospy
import argparse
import sys

from soma_manager.data_manager import SOMADataManager

if __name__=="__main__":

    parser = argparse.ArgumentParser(prog='data_manager_node.py')
    parser.add_argument("--object_db_name", nargs='?', help='Name of the mongo database' , default="somadata")
    parser.add_argument('--object_collection_name', nargs='?', help='Name of the object collection', default="object")
    parser.add_argument('--roi_collection_name', nargs='?', help='Name of the roi collection',default="roi")

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("soma_data_manager")

    rospy.loginfo("Running SOMA Data Manager (object db: %s, object_collection: %s, roi_collection: %s)", args.object_db_name, args.object_collection_name,args.roi_collection_name)
    SOMADataManager(args.object_db_name,args.object_collection_name,args.roi_collection_name)
