#!/usr/bin/env python

import rospy
import argparse
import sys

from soma_manager.data_manager import SOMADataManager

if __name__=="__main__":

    parser = argparse.ArgumentParser(prog='data_manager_node.py')
    parser.add_argument("--object_db_name", nargs='?', help='Name of the database')
    parser.add_argument('--object_collection_name', nargs='?', help='Name of the collection')

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("soma_data_manager")
    if args.object_db_name is not None:
        if args.object_collection_name is not None:
           rospy.loginfo("Running SOMA data manager (object dbname: %s, object collection_name: %s)", args.object_db_name, args.object_collection_name)
           SOMADataManager(args.object_db_name,args.object_collection_name)
        else:
            rospy.loginfo("Running SOMA data manager (object dbname: %s, object collection_name: object)", args.object_db_name)
            SOMADataManager(args.object_db_name)
    else:
        rospy.loginfo("Running SOMA data manager (object dbname: somadata, object collection_name: object)")
        SOMADataManager()
