#!/usr/bin/env python

import sys
import rospy
from relational_learner.srv import *
import imports.obtain_trajectories as ot
from human_trajectory.msg import Trajectories

import card_checking.msg


def novelty_client(Trajectory):
    
    rospy.wait_for_service('/novelty_detection')
    proxy = rospy.ServiceProxy('/novelty_detection', NoveltyDetection)  
    # req = NoveltyDetectionRequest(Trajectory)
    # ret = proxy(req)
    return proxy(Trajectory)

#human_trajectory/ Trajectory[]
def listener():
    rospy.Subscriber("/human_trajectories/trajectories", Trajectories, callback)
    rospy.sleep(0.1)

ret=None

def callback(msg):
    if len(msg.trajectories) > 0:
        ret = novelty_client(msg.trajectories[0])
	print "RET = ", ret
	
if __name__ == "__main__":
    rospy.init_node('novelty_client')

    listener()
    while not rospy.is_shutdown():
        if ret !=None:
	    print ret   

    rospy.loginfo('-- Card Checking Client Active --')

    cac = CheckCardClient()

    print cac
    sys.exit(1)
    cac.demandspawn()


    rospy.spin()
























