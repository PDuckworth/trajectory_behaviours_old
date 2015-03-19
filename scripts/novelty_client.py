#!/usr/bin/env python

import sys
import rospy
from relational_learner.srv import *
import imports.obtain_trajectories as ot
from human_trajectory.msg import Trajectories


def novelty_client(Trajectory):
    
    rospy.wait_for_service('novelty_detection')
    proxy = rospy.ServiceProxy('novelty_detection', NoveltyDetection)  
    req = NoveltyDetectionRequest(Trajectory)
    ret = proxy(req)
    return ret

#human_trajectory/ Trajectory[]
def listener():
    rospy.Subscriber("/human_trajectories/trajectories", Trajectories, callback)
    rospy.sleep(0.1)

def callback(msg):
    if len(msg.trajectories) > 0:
        novelty_client(msg.trajectories[0])

if __name__ == "__main__":
    rospy.init_node('novelty_client')

    listener()
    """2. Call the novelty server"""
    # for i, traj in enumerate(trajectories.trajectories):
    

    """3. Call the approach action lib server"""
    #Get code from Jay


    """Obtain one trajectory from mongodb
       Pass it to novelty_client as a test case!

       This comes from geo_store and hence is in long/lat! 
    """
    #query = '''{"uuid": "328e2f8c-6147-5525-93c4-1b281887623b"}''' ## ROI 12
    #query = '''{"uuid": "23981628-462f-57cb-8edf-a1ce8bce88d2"}'''  ## ROI 14
    
    #q = ot.query_trajectories(query)
    #print q.res.trajectories.trajectories[0].trajectory[0].pose.position
    #print type(mimic_trajectory_msg)
    #mimic_trajectory_msg = q.res.trajectories.trajectories[0]
    #ret = novelty_client(mimic_trajectory_msg)
    
    #print ret


    rospy.spin()
























