#!/usr/bin/env python

"""Queries Mongodb for Object and Trajectory data"""

__author__      = "Paul Duckworth"
__copyright__   = "Copyright 2015, University of Leeds"


import rospy
import pymongo
import os, sys, time, copy
import logging
import itertools
import numpy as np
import cPickle as pickle
import random
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point

from geometry_msgs.msg import Pose
#from trajectory import Trajectory, TrajectoryAnalyzer

from human_trajectory.msg import Trajectory
from human_trajectory.msg import Trajectories
from soma_trajectory.srv import TrajectoryQuery, TrajectoryQueryRequest, TrajectoryQueryResponse

from query_examples import get_query

logger = logging.getLogger('obtain_trajectories')

#**************************************************************#
#             Obtain Objects and Trajectories                  #
#**************************************************************#

class query_roi():

    def __init__(self):
        self.all_rois = dict()
        host = rospy.get_param("mongodb_host")
        port = rospy.get_param("mongodb_port")
        self._client = pymongo.MongoClient(host, port)
        self._retrieve_logs()

    def _retrieve_logs(self):
        logs = self._client.message_store.soma.find()

        for log in logs:
            #print "log: " + repr(log)
            for i, _id in enumerate(log['id']):
                #print "_id: " + repr(_id)
                #print "log pos: " + repr(log['pose'])
                #print "obj: " + repr(log['type'])

                x = log['pose']['position']['x']
                y = log['pose']['position']['y']
                z = log['pose']['position']['z']  
                obj_instance = log['type'] + '_' + log['id']
                #print obj_instance

                if _id not in self.all_objects:
                    self.all_objects[obj_instance] = [(x,y,z)]
                else:
                    self.all_objects[obj_instance] = [(x,y,z)]
        return


def get_soma_roi():
    return query_roi()


class query_objects():

    def __init__(self):
        self.all_objects = dict()
        host = rospy.get_param("mongodb_host")
        port = rospy.get_param("mongodb_port")
        self._client = pymongo.MongoClient(host, port)
        self._retrieve_logs()

    def _retrieve_logs(self):
        logs = self._client.message_store.soma.find()

        for log in logs:
            #print "log: " + repr(log)
            for i, _id in enumerate(log['id']):
                #print "_id: " + repr(_id)
                #print "log pos: " + repr(log['pose'])
                #print "obj: " + repr(log['type'])

                x = log['pose']['position']['x']
                y = log['pose']['position']['y']
                z = log['pose']['position']['z']  
                obj_instance = log['type'] + '_' + log['id']
                #print obj_instance

                if _id not in self.all_objects:
                    self.all_objects[obj_instance] = [(x,y,z)]
                else:
                    self.all_objects[obj_instance] = [(x,y,z)]
        return


    def check(self):
        print 'All objects in SOMA:'
        for i, object in enumerate(self.all_objects):
            print repr(i) + ",  " + repr(object) + ",  " + repr(self.all_objects[object]) 
        print 'All objects Loaded\n'


def objects_in_scene():
    return query_objects()




class query_trajectories():

    def __init__(self):
        self.traj_poses = dict()
        host = rospy.get_param("mongodb_host")
        port = rospy.get_param("mongodb_port")
        self._client = pymongo.MongoClient(host, port)
        self._retrieve_logs()

    def _retrieve_logs(self):
        logs = self._client.message_store.people_perception.find()   # Need the Trajectory database in here

        for log in logs:
            #print "log: " + repr(log)
            for i, uuid in enumerate(log['uuid']):
                print "uuid: " + repr(uuid)
                print "log pos: " + repr(log['pose'])
                print "log time: " + repr(log['start_time']) + ",  "  + repr(log['end_time'])
                print "header: " + repr(log['header']['stamp'])

                if uuid not in self.traj_poses:
                    self.traj_poses[uuid] = log['trajectory']
                else:
                   print "SAME ID Twice in Mongodb"
        return

    def check(self):
        cnt = 0
        for uuid, poses in self.items():
            if cnt == 0:
                print "example trajectory:"
                print uuid, poses
            cnt+=1

        print repr(cnt) + ' Trajectories Loaded'



class QueryClient():
    """Query the Trajectory Objects in mongodb directly"""
    def __init__(self):
        service_name = 'trajectory_query'
        rospy.wait_for_service(service_name)
        self.ser = rospy.ServiceProxy(service_name, TrajectoryQuery)

    def query(self, query, vis = False):
        try:
            req = TrajectoryQueryRequest()
            req.query = query
            req.visualize = vis
            res = self.ser(req)
            return res
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)






def pickle_load_trajectories(traj_dir, pickle_file):
    trajectories = pickle.load(open(os.path.join(traj_dir, pickle_file)))
    traj_poses = {}
    for uuid, traj in trajectories.items():
        #print uuid, traj
        #print type(traj.pose)
        #print type(traj.pose[0])

        traj_poses[uuid] = []
        for pose in traj.pose:
            x =  pose['position']['x']
            y = pose['position']['y']
            z = pose['position']['z']
            traj_poses[uuid].append((x,y,z))
        #print repr(len(traj_xy[uuid])) + " poses in trajectory"
    return traj_poses

def check_traj(traj_poses):
    cnt = 0
    for uuid, poses in traj_poses.items():
        if cnt == 0:
            print "example trajectory:"
            print "  " + repr(uuid) + ",  " + repr(poses)
        cnt+=1
    print repr(cnt) + ' Trajectories Loaded\n'




def get_trajectories(load_method = 'pickle', traj_dir=None, pickle_file=None):

    if load_method == 'pickle':
        traj_poses = pickle_load_trajectories(traj_dir, pickle_file)
        return traj_poses

    elif load_method == 'mongo':

        client = QueryClient()
        query = get_query()
        logger.info("Query: %s" % query )
        start = time.time()
        res = client.query(query, True)
        print "time to query = " + repr(time.time() - start)

        logger.info("Result: %s trajectories" % len(res.trajectories.trajectories))   
        print "num traj returned = " + repr(len(res.trajectories.trajectories))

        return res.trajectories.trajectories





def processFeedback(feedback):
    p = feedback.pose.position
    print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)
   
    p = feedback.pose.orientation
    print p.x, p.y,p.z,p.w




def select_landmark_poses(input_data):
    """Input a list of all poses. Output a random selection of landmarks"""

    landmark_poses = []
    for i in xrange(10):
        pose= random.choice(input_data)
        landmark_poses.append(pose)

    return landmark_poses



class Landmarks():

    def __init__(self, poses):
        rospy.init_node("simple_marker")
        # create an interactive marker server on the topic namespace simple_marker
        self._server = InteractiveMarkerServer("simple_marker")

        self.poses_landmarks = poses
        self.visualize_landmarks()
      


    def visualize_landmarks(self):
        """Create an interactive marker per landmark"""

        for i, pose in enumerate(self.poses_landmarks):
            name = "landmark_" + repr(i)
            int_marker = self.create_marker(pose, name)

        
    def create_marker(self, pose, name, interactive=False):
        print name, pose
        (x,y,z) = pose

        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/map"
        int_marker.name = "%s" %name
        int_marker.description = "Simple 1-DOF Control"

        # create a grey box marker
        box_marker = Marker()
        box_marker.type = Marker.ARROW

        box_marker.scale.x = 1
        box_marker.scale.y = 0.1
        box_marker.scale.z = 0.1
    
        box_marker.pose.position = Point(x, y, 1)
        box_marker.pose.orientation.x = -8.02139854539e-10
        box_marker.pose.orientation.y = 0.695570290089
        box_marker.pose.orientation.z = 1.66903157961e-10
        box_marker.pose.orientation.w = 0.695570290089
        
        box_marker.color.r = 8.0
        box_marker.color.g = 0.1
        box_marker.color.b = 0.1
        box_marker.color.a = 0.7

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append( box_marker )
        int_marker.controls.append( box_control )

        if interactive:
            rotate_control = InteractiveMarkerControl()
            rotate_control.name = "move_x"
            rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(rotate_control);


            #To create a control that rotates the arrows:
            control = InteractiveMarkerControl()
            control.orientation.x = 0
            control.orientation.y = 0
            control.orientation.z = 1
            control.orientation.w = 1
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(control)

        self._server.insert(int_marker, processFeedback)

        # 'commit' changes and send to all clients
        self._server.applyChanges()
        return int_marker



if __name__ == "__main__":
    global __out
    __out = True

    base_data_dir = '/home/strands/STRANDS/'
    options_file = os.path.join(base_data_dir + 'options.txt')
    traj_dir = os.path.join(base_data_dir, 'trajectory_dump')


    """NEED TO QUERY REGIONS OF INTEREST!!! """
    soma_rois = ['ROI_1']




    """TRAJECTORIES"""
    client = QueryClient()
    query = get_query()
    rospy.loginfo("Query: %s" % query )
    start = time.time()
    res = client.query(query, True)
    print "time to query = " + repr(time.time() - start)

    rospy.loginfo("Result: %s trajectories" % len(res.trajectories.trajectories))
    rospy.loginfo("Type of first trajectory is: %s" % type(res.trajectories.trajectories[0]))
    rospy.loginfo("Trajectory.trajectory[0]: %s" % (res.trajectories.trajectories[0].trajectory[0]))
    
    print "num traj returned = " + repr(len(res.trajectories.trajectories))
    trajectory_poses={}

    cnt=0
    for roi in soma_rois:
        trajectory_poses[roi] = {}

        for trajectory in res.trajectories.trajectories:
            trajectory_poses[roi][trajectory.uuid] = []
                
            for entry in trajectory.trajectory:
                x=entry.pose.position.x
                y=entry.pose.position.y
                z=entry.pose.position.z
                trajectory_poses[roi][trajectory.uuid].append((x,y,z))

    print "number of unique traj returned = " + repr(len(trajectory_poses['ROI_1']))

    raw_input("Press enter to continue")

    #Need a method for querrying objects in ROI
    #objects = objects_in_scene()
    #if __out: objects.check()     

    #Not needed if trakectories per region are being queried
    #traj_poses = get_trajectories('pickle', traj_dir, 'reduced_traj.p')
    #if __out: check_traj(traj_poses) 
  
    """NEED TO ADD "PER ROI" AROUND THE LANDMARK GENERATION"""
    """Create Landmark pins at randomly selected poses from all the trajectory data"""      
    all_poses = list(itertools.chain.from_iterable(trajectory_poses['ROI_1'].values()))
    print "number of poses in total = " +repr(len(all_poses))

    landmarks = select_landmark_poses(all_poses)
    print "landmarks = " + repr(landmarks)

    pins = Landmarks(landmarks)

    rospy.spin()  
