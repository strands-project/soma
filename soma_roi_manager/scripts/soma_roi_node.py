#!/usr/bin/env python

import rospy
import argparse
import sys

from soma_roi_manager.soma_roi import SOMAROIManager

if __name__=="__main__":

    # TODO: add list command

    parser = argparse.ArgumentParser(prog='soma_roi.py')
 
    parser.add_argument("conf", nargs=1, help='Name of the object configuration')
    parser.add_argument('-t', metavar='config-file')
    parser.add_argument('--db_name', help='Name of the roi db',default="somadata")
    parser.add_argument('--collection_name', help='Name of the roi collection',default="roi")

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("soma_roi")
    
    SOMAROIManager(args.conf[0],args.t,args.db_name,args.collection_name)
