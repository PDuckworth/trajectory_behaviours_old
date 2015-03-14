#!/usr/bin/env python

import sys
import rospy
from relational_learner.srv import *
import imports.obtain_trajectories as ot


def novelty_client(Trajectory):
    rospy.wait_for_service('novelty_detection')
    proxy = rospy.ServiceProxy('novelty_detection', NoveltyDetection)
        
    req = NoveltyDetectionRequest(Trajectory)
    ret = proxy(req)
    return ret



if __name__ == "__main__":

    """Obtain one trajectory from mongodb
       Pass it to novelty_client as a test case!

       This comes from geo_store and hence is in long/lat! 
    """
    #query = '''{"uuid": "328e2f8c-6147-5525-93c4-1b281887623b"}'''
    query = '''{"uuid": "23981628-462f-57cb-8edf-a1ce8bce88d2"}'''
    
    q = ot.query_trajectories(query)
    #print q.res.trajectories.trajectories[0].trajectory[0].pose.position
    #print type(mimic_trajectory_msg)
    mimic_trajectory_msg = q.res.trajectories.trajectories[0]

    print novelty_client(mimic_trajectory_msg)

