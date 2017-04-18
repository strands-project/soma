#!/usr/bin/env python

import rospy
from soma_trajectory.srv import SOMAQueryTrajectories, SOMAQueryTrajectoriesRequest


def average_velocity(traj):
    start = traj.start_time
    end = traj.end_time
    delta = float((end-start).secs + 0.000000001 * (end-start).nsecs)
    avg_vel = 0.0
    if delta != 0.0:
        avg_vel = traj.length / delta
    return avg_vel


if __name__ == "__main__":
    rospy.init_node("soma_trajectory_visualisation")
    rospy.loginfo("Wait for soma/query_trajectories service...")
    srv = rospy.ServiceProxy("soma/query_trajectories", SOMAQueryTrajectories)
    srv.wait_for_service()
    rospy.loginfo("Requesting visualisation for all trajectories...")
    res = srv(
        SOMAQueryTrajectoriesRequest(
            visualize=True
            # , region_id="1", region_config="g4s_novelty"
        )
    )
    trajectories = res.trajectories

    average_length = 0.0
    longest_length = -1.0
    short_trajectories = 0
    average_vel = 0.0
    highest_vel = -1.0

    for traj in trajectories:
        average_length += float(traj.length)
        avg_vel = average_velocity(traj)
        average_vel += average_velocity(traj)
        if traj.length < 1:
            short_trajectories += 1
        if longest_length < traj.length:
            longest_length = traj.length
        if highest_vel < avg_vel:
            highest_vel = avg_vel

    average_length /= float(len(trajectories))
    average_vel /= float(len(trajectories))
    rospy.loginfo("Average length of tracks is " + str(average_length))
    rospy.loginfo("Longest length of tracks is " + str(longest_length))
    rospy.loginfo("Short trajectories are " + str(short_trajectories))
    rospy.loginfo("Average velocity of tracks is " + str(average_vel))
    rospy.loginfo("Highest velocity of tracks is " + str(highest_vel))
    rospy.loginfo("Total number of trajectories: %d" % len(trajectories))
    raw_input("Press 'Enter' to exit.")
