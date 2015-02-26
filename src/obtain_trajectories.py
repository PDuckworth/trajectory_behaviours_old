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
from scipy import spatial
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
                    self.all_objects[obj_instance] = (x,y,z)
                else:
                    self.all_objects[obj_instance] = (x,y,z)
        return


    def check(self):
        print 'All objects in SOMA:'
        for i, key in enumerate(self.all_objects):
            print repr(i) + ",  " + repr(key) + ",  " + repr(self.all_objects[key]) 
        print repr(len(self.all_objects)) + ' objects Loaded.\n'


def objects_in_scene():
    return query_objects()






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
            print "  " + repr(uuid) + ",  " + repr(poses) + ". LENGTH = " +repr(len(poses))
        cnt+=1
    print repr(cnt) + ' Trajectories Loaded\n'



def get_trajectories(load_method, traj_dir=None, pickle_file=None):

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

        trajectory_poses={}
        for trajectory in res.trajectories.trajectories:
            trajectory_poses[trajectory.uuid] = []
                
            for entry in trajectory.trajectory:
                x=entry.pose.position.x
                y=entry.pose.position.y
                z=entry.pose.position.z
                trajectory_poses[trajectory.uuid].append((x,y,z))

        return trajectory_poses





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

        # create an interactive marker server on the topic namespace landmark_markers
        self._server = InteractiveMarkerServer("landmark_markers")
        self.poses = poses
        self.poses_landmarks = {}
        self.visualize_landmarks()
      


    def visualize_landmarks(self):
        """Create an interactive marker per landmark"""

        for i, pose in enumerate(self.poses):
            name = "landmark" + repr(i)
            self.poses_landmarks[name] = pose
            int_marker = self.create_marker(pose, name)

        
    def create_marker(self, pose, name, interactive=False):
        #print name, pose
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




def trajectory_object_dist(objects, trajectory_poses):
    uuids=trajectory_poses.keys()
    object_ids=objects.keys()

    print repr(len(uuids)) + " trajectories.  "+ repr(len(object_ids)) + " objects. Nearest 4 selected."

    cnt=0
    object_distances={}
    distance_objects={}
    for (uuid, obj) in itertools.product(uuids, object_ids):
        #object_distances[(uuid, obj)] = [] #No need for list, if only taking init_pose
        #print (uuid, obj)

        #Just select the first trajectory pose for now :)
        traj_init_pose = trajectory_poses[uuid][0]
        object_pose = objects[obj] #Objects only have one pose
        dist = spatial.distance.pdist([traj_init_pose, object_pose], 'euclidean')

        if uuid not in object_distances:
            object_distances[uuid] = {}
            distance_objects[uuid]={}

        object_distances[uuid][obj] = dist[0]
        distance_objects[uuid][dist[0]]= obj
        if len(object_distances[uuid]) != len(distance_objects[uuid]):
            print "multiple objects exactly the same distance from trajectory: " + repr(uuid)
            print "object: " + repr(obj)
            sys.exit(1)
        #print cnt
        cnt+=1

    cnt=0
    closest_objects = {}
    for uuid, dist_objs in distance_objects.items():
        keys = dist_objs.keys()
        keys.sort()

        #select closest 4 objects or landmarks
        closest_dists = keys[0:4]
        closest_objects[uuid]={}
        for dist in closest_dists:
            #print dist, dist_objs[dist]
            obj = dist_objs[dist]
            closest_objects[uuid][obj] = objects[obj]
        #print cnt
        cnt+=1

    return closest_objects




if __name__ == "__main__":
    global __out
    __out = False

    base_data_dir = '/home/strands/STRANDS/'
    options_file = os.path.join(base_data_dir + 'options.txt')
    traj_dir = os.path.join(base_data_dir, 'trajectory_dump')


    trajectory_poses = get_trajectories('mongo', traj_dir)
    print "number of unique traj returned = " + repr(len(trajectory_poses))

    """Need a method for querrying objects in ROI"""
    objects = objects_in_scene()
    if __out: objects.check()     

    #Not needed if trakectories per region are being queried
    #traj_poses = get_trajectories('pickle', traj_dir, 'reduced_traj.p')
    #if __out: check_traj(traj_poses) 
  
    """Create Landmark pins at randomly selected poses from all the trajectory data"""      
    all_poses = list(itertools.chain.from_iterable(trajectory_poses.values()))
    print "number of poses in total = " +repr(len(all_poses))

    landmarks = select_landmark_poses(all_poses)
    if __out: print "landmarks = " + repr(landmarks)
    pins = Landmarks(landmarks)
    
    objects_per_trajectory = trajectory_object_dist(objects.all_objects, trajectory_poses)
    landmarks_per_trajectory = trajectory_object_dist(pins.poses_landmarks, trajectory_poses)
    if __out: print objects_per_trajectory['7d638405-b2f8-55ce-b593-efa8e3f2ff2e']
    if __out: print landmarks_per_trajectory['7d638405-b2f8-55ce-b593-efa8e3f2ff2e']

    rospy.spin()  
