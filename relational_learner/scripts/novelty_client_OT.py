#!/usr/bin/env python

import sys
import rospy
from relational_learner.srv import *
import relational_learner.obtain_trajectories as ot

class NoveltyClient(object):

    def __init__(self):
        self.ret = None
        self.uuid = ''
        self.pose = None

    def novelty_client(self, Trajectory):
        rospy.wait_for_service('/novelty_detection')
        proxy = rospy.ServiceProxy('/novelty_detection', NoveltyDetection)  
        req = NoveltyDetectionRequest(Trajectory)
        ret = proxy(req)
        return ret

    def listener(self):
        rospy.Subscriber("/human_trajectories/trajectories/batch", Trajectories, self.callback)
        rospy.sleep(0.1)

    def callback(self, msg):
        if len(msg.trajectories) > 0:
            self.uuid = msg.trajectories[0].uuid
            self.pose = msg.trajectories[0].trajectory[-1].pose
            self.ret = self.novelty_client(msg.trajectories[0])







if __name__ == "__main__":
    rospy.init_node('novelty_client')
    """
    # Stitch together mini-batch trajectory msg if the uuid matches.
    # Stitch using the .seq 

    """
    nc = NoveltyClient()

   
    """Obtain one trajectory from mongodb
       Pass it to novelty_client as a test case!

       This comes from geo_store and hence is in long/lat! 
    """
    query = '''{"uuid": "328e2f8c-6147-5525-93c4-1b281887623b"}''' ## ROI 12
    q = ot.query_trajectories(query)
    mimic_trajectory_msg = q.res.trajectories.trajectories[0]

    print query

    ret = nc.novelty_client(mimic_trajectory_msg)
    
    print ret

    """
    # Publish nt.ret on "all_novelty" topic
    # A novelty_decision node, to Listen to ret, and run some logic to decide which to approach
    # Publish this decision on "super_novel" topic
    # Jay then calls CardCheckClient (not me :)
    """
    

    #rospy.spin()
























