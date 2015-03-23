#!/usr/bin/env python

import sys
import rospy
from relational_learner.srv import *
import relational_learner.obtain_trajectories as ot
from human_trajectory.msg import Trajectories

import card_check.card_check_client as ccc
import card_checking.msg

class NoveltyClient(object):

    def __init__(self):
        self.ret = None
        self.uuid = ''
        self.pose = None

    def novelty_client(self, Trajectory):
        rospy.wait_for_service('/novelty_detection')
        proxy = rospy.ServiceProxy('/novelty_detection', NoveltyDetection)  
        # req = NoveltyDetectionRequest(Trajectory)
        # ret = proxy(req)
        return proxy(Trajectory)

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
    nc = NoveltyClient()
    nc.listener()
  

    # Stitch together mini-batch trajectory msg if the uuid matches.
    # Stitch using the .seq 

    while not rospy.is_shutdown():
        if nc.ret !=None:
            print "\nRESULTS = ", nc.ret

            # Publish nt.ret on "all_novelty" topic
            # A novelty_decision node, to Listen to ret, and run some logic to decide which to approach
            # Publish this decision on "super_novel" topic
            # Jay then calls CardCheckClient (not me :)
            rospy.loginfo('-- Card Checking Client Active --')
            cac = ccc.CheckCardClient()
            goal = card_checking.msg.CardCheckGoal()
            goal.id = nc.uuid
            goal.pose = nc.pose
            cac.demandspawn(goal)
 





    rospy.spin()
























