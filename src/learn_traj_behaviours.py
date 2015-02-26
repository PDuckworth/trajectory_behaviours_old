#!/usr/bin/env python

"""compute_spatial_relations.py: Computes spatial relations."""

__author__      = "Paul Duckworth"
__copyright__   = "Copyright 2014, University of Leeds"

import rospy
import pymongo
import os, sys, time
import logging
import argparse
import itertools
import getpass

import numpy as np
import pyrr
from scipy import spatial
import cPickle as pickle

from geometry_msgs.msg import Pose
from trajectory import Trajectory, TrajectoryAnalyzer

from qsrlib.qsrlib import QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace
from qsrlib_ros.qsrlib_ros_client import QSRlib_ROS_Client

from query_examples import get_query
import obtain_trajectories as ot

# create logger with 'spam_application'
logger = logging.getLogger('traj_learner')
logger.setLevel(logging.DEBUG)
# create file handler which logs even debug messages
if not os.path.isdir('/tmp/logs'):
    os.system('mkdir -p /tmp/logs')
fh = logging.FileHandler('/tmp/logs/traj_learner.log')
fh.setLevel(logging.DEBUG)
# create console handler with a higher log level
ch = logging.StreamHandler()
ch.setLevel(logging.INFO)
# create formatter and add it to the handlers
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
fh.setFormatter(formatter)
ch.setFormatter(formatter)
# add the handlers to the logger
logger.addHandler(fh)
logger.addHandler(ch)

#**************************************************************#
#        Implement QSR Lib on objects and trajectories         #
#**************************************************************#

options = {"rcc3": "rcc3_rectangle_bounding_boxes_2d",
           "qtcb": "qtc_b_simplified",
           "qtcc": "qtc_c_simplified",
           "qtcbc": "qtc_bc_simplified",
           "rcc3a": "rcc3_rectangle_bounding_boxes_2d"}


def get_qsrlib_world(uuid, objects, poses, params):
    (qsr, _q,v,n) =  params

    #convert q from string to float
    q = float('0.' + _q.split('_')[1])

    o1 = []          #object 1 is always the trajectory
    o2_dic = {}      #object 2 is always the SOMA objects
    worlds = {}

    for obj in objects:
        o2_dic[obj] = []
        key = (uuid, obj)
        if __out: print "worlds key: " + repr(key)
        worlds[key] = World_Trace()

    for frame, (x,y,z) in enumerate(poses):
        if __out: print "Frame # " + repr(frame)

        o1.append(Object_State(name="trajectory", timestamp = frame, x=x, y=y, \
                quantisation_factor=q, validate=v, no_collapse=n))
        if __out: print "   added trajectory pose" + repr(frame) + " to o1."

        for obj in objects:            
            (x,y,z) = objects[obj]
            if __out: print "   added object = " + repr(obj) + "  to o2"

            o2_dic[obj].append(Object_State(name=obj, timestamp = frame, x=x, y=y, \
                quantisation_factor=q, validate=v, no_collapse=n))
    
        if __out: print "number of traj poses= " + repr(len(o1))
        if __out: print "paired with objects: " + repr(o2_dic.keys())
    
    for obj, o2 in o2_dic.items():
        worlds[(uuid, obj)].add_object_state_series(o1)
        worlds[(uuid, obj)].add_object_state_series(o2)
    return worlds


def apply_qsr_lib(objects_per_trajectory, trajectories, params, qsr_file):
    "Formats the object and trajectories and passes to qsrlib"

    spatial_relations = {}
    which_qsr = options[params[0]]
    if __out: print "Qsr: " + repr(which_qsr)
    if __out: print "With params: " + repr(params)

    #client_node = rospy.init_node("qsr_lib_ros_client")    

    for uuid, poses in trajectories.items():
        objects = objects_per_trajectory[uuid]

        if __out: print repr(uuid) + ",  #poses = " + repr(len(poses))
        if __out: print "object file: " + repr(objects)

        worlds = get_qsrlib_world(uuid, objects, poses, params)
        spatial_relations[uuid] = {}

        for (uuid, obj), world in worlds.items():
            qsrlib_request_message = QSRlib_Request_Message(which_qsr=which_qsr, \
                   input_data=world, include_missing_data=True)
            cln = QSRlib_ROS_Client()
            req = cln.make_ros_request_message(qsrlib_request_message)
            res = cln.request_qsrs(req)
            out = pickle.loads(res.data)
            if __out: print("Request was made at ", str(out.timestamp_request_made))

            for t in out.qsrs.get_sorted_timestamps():
                if t not in spatial_relations[uuid]:       
                    spatial_relations[uuid][t] = {}
                
                relations = str(out.qsrs.trace[t].qsrs.values()[0].qsr)
                spatial_relations[uuid][t][(uuid, obj)] = relations
                #print spatial_relations[uuid][t][(uuid, obj)]

    #spatial_relations is a list of dictionaries. One per Trajectory, key=(uuid,objs). values=poses.
    pickle.dump(spatial_relations, open(qsr_file, 'w'))
    return spatial_relations



#**************************************************************#
#     Compute Episode Representation of the Spatial Relations  #
#**************************************************************#

def generate_episode_data(spatial_relations, output_file, noise_thres=3):
    from data_processing_utils import  compute_episodes, filter_intervals

    cnt=0
    all_episodes = {}
    NOISE_THRESHOLD = noise_thres

    for uuid in spatial_relations:
        key, epi  = compute_episodes(spatial_relations[uuid], cnt, __out)

        # Filter the episods to remove very short transitions that are noise
        fepi = {}
        for e in epi:
            fepi[e] = filter_intervals(epi[e], NOISE_THRESHOLD)

        # Add filtered episodes to all_episodes
        all_episodes[key] = fepi
        cnt+=1
    
    pickle.dump(all_episodes, open(output_file,'w'))
    logger.info('3. Episode Data Generated and saved to:\n' + output_file) 
    return all_episodes


#**************************************************************#
#      Create Activity Graphs For Each Trajectory Instance     #
#**************************************************************#

def generate_graph_data(episodes, data_dir, params):
    from Activity_Graph import Activity_Graph

    AG_out_file = os.path.join(data_dir + 'activity_graphs_' + tag + '.p')
    if __out: print AG_out_file

    activity_graphs = {}
    logger.info('4. Generating activity graphs from episodes')

    cnt=0
    for episodes_file in episodes:  

        uuid, start, end = episodes_file.split('__')        
        logger.info('4. Processing for graphlets: ' + episodes_file)

        episodes_dict = episodes[episodes_file]
        episodes_list = list(itertools.chain.from_iterable(episodes_dict.values()))

        activity_graphs[episodes_file] = Activity_Graph(episodes_list, params)
        activity_graphs[episodes_file].get_valid_graphlets()

                
        if __out and cnt == 1:
            activity_graphs[episodes_file].graph2dot('/tmp/act_ntc.dot', False)
            print "episode file 1: " +repr(episodes_file)
            print activity_graphs[episodes_file].graph
            print "episode file 2: " +repr(episodes_file)
            print activity_graphs[episodes_file].valid_graphlets
            #sys.exit(1)   
        cnt+=1
        print cnt
      
    pickle.dump(activity_graphs, open(AG_out_file,'w')) 
    logger.info('4. Activity Graph Data Generated and saved to:\n' + AG_out_file) 
    return


def generate_feature_space(data_dir, tag):
    
    AG_out_file = os.path.join(data_dir + 'activity_graphs_' + tag + '.p')
    activity_graphs = pickle.load(open(AG_out_file))
    
    logger.info('5. Generating codebook')
    code_book, graphlet_book = [], []
    code_book_set, graphlet_book_set = set([]), set([])
    for episodes_file in activity_graphs: 

        #for window in activity_graphs[episodes_file].graphlet_hash_cnts:     #Loop through windows, if multiple windows
        window = activity_graphs[episodes_file].graphlet_hash_cnts.keys()[0]

        for ghash in activity_graphs[episodes_file].graphlet_hash_cnts[window]:
            if ghash not in code_book_set:
                code_book_set.add(ghash)
                graphlet_book_set.add(activity_graphs[episodes_file].valid_graphlets[window][ghash])
    code_book.extend(code_book_set)
    graphlet_book.extend(graphlet_book_set)   

    print "len of code book: " + repr(len(code_book))
    if len(code_book) != len(graphlet_book): 
        print "BOOK OF HASHES DOES NOT EQUAL BOOK OF ACTIVITY GRAPHS"
        sys.exit(1)

    logger.info('5. Generating codebook FINISHED')
    logger.info('5. Generating features')

    cnt = 0
    X_source_U = []
    #Histograms are Windowed dictionaries of histograms 
    for episodes_file in activity_graphs:
        print cnt, episodes_file
        histogram = activity_graphs[episodes_file].get_histogram(code_book)
        X_source_U.append(histogram)
        cnt+=1
        if cnt ==1:
            key = activity_graphs[episodes_file].graphlet_hash_cnts.keys()[0]
            print "KEY = " + repr(key)
            print "hash counts: " + repr(activity_graphs[episodes_file].graphlet_hash_cnts[key].values())
            print "sum of hash counts: " + repr(sum(activity_graphs[episodes_file].graphlet_hash_cnts[key].values()))
            print "sum of histogram: " + repr(sum(histogram))
    
    logger.info('Generating features FINISHED')
    logger.info('Saving all experiment data')       
    
    feature_space = (code_book, graphlet_book, X_source_U)

    feature_space_out_file = os.path.join(data_dir + 'feature_space_' + tag + '.p')
    pickle.dump(feature_space, open(feature_space_out_file, 'w'))
    print "\nall graph and histogram data written to: \n" + repr(data_dir) 
    
    logger.info('Done')
    return



def check_dir(directory):
    if not os.path.isdir(directory):
        os.system('mkdir -p ' + directory)
    return


def qsr_setup(data_dir, params, date):
    params_tag = map(str, params)
    params_tag = '__'.join(params_tag)
    qsr_tag = params_tag + date

    qsr_dir = os.path.join(data_dir, 'qsr_dump/')
    check_dir(qsr_dir)
    return qsr_dir, qsr_tag


def AG_setup(my_data, date):
    params_str = (my_data['MIN_ROWS'], my_data['MAX_ROWS'], my_data['MAX_EPI'], my_data['num_cores'])#
    params = []
    for x in params_str:
        params.append(int(x)) if x != 'None' else params.append(None)

    params_tag = map(str, params)
    params_tag = '_'.join(params_tag)
    tag = params_tag + date

    return params, tag







if __name__ == "__main__":
    global __out
    rospy.init_node("trajectory_learner")

    parser = argparse.ArgumentParser()
    parser.add_argument("sections", help="chose what section of code to run", type=str)
    args = parser.parse_args()
 
    run_options = {'1': "Get Objects & Trajectories",
                   '2': "Apply QSR Lib",
                   '3': "Generate Episodes",
                   '4': "Activity Graphs/Graphlets",
                   '5': "Generate Code Book and Histograms",
                   '6': "Learn unsupervised behaviours"}

    sections = args.sections
    sections_dic={}

    try:
        print "\n************************************************"
        for i in sections:
            if i not in (" ",","): sections_dic[i] = run_options[i]
        sections=sections_dic.keys()
        sections.sort()
        for key in sections:
            print "Selected: " + repr(key) + ". Running: " + repr(sections_dic[key])
        print "************************************************"
    except KeyError:
        print("Run Option not found")
        print "************************************************"
        print "Current working selection: "
        keys=sections_dic.keys()
        keys.sort()
        for key in keys:
            print "Selected: " + repr(key) + ". Running: " + repr(sections_dic[key])
        print "************************************************"
        sys.exit(1)
    
    user = getpass.getuser()
    base_data_dir = os.path.join('/home/' + user + '/STRANDS/')
    options_file = os.path.join(base_data_dir + 'options.txt')

    my_data = {}
    gen = (line for line in open(options_file) if line[0] != '#')
    for line in gen:
        line = line.replace('=', ' ').replace('\n', '')
        var, value = line.split(' ')
        key = var
        my_data [key] = value
    date = my_data['date']
    qsr_params = (my_data['qsr'],my_data['q'],my_data['v'],my_data['n'])

    #**************************************************************#
    #             Obtain Objects and Trajectories                  #
    #**************************************************************#
    __out = False
    if '1' in sections_dic:
        
        logger.info('1. Loading Objects and Trajectory Data')
        traj_dir = os.path.join(base_data_dir, 'trajectory_dump')

        #regions_of_interest = get_soma_roi()

        objects = ot.objects_in_scene()
        if __out: objects.check()
   
        #traj_poses = get_trajectories('pickle', traj_dir, 'reduced_traj.p')
        trajectory_poses = ot.get_trajectories('mongo', traj_dir)
        if __out: ot.check_traj(trajectory_poses) 
    
        all_poses = list(itertools.chain.from_iterable(trajectory_poses.values()))
        if __out: print "number of poses in total = " +repr(len(all_poses))
        landmarks = ot.select_landmark_poses(all_poses)
     
        pins = ot.Landmarks(landmarks)
        if __out: print "landmark poses = " + repr(landmarks)
        if __out: print pins.poses_landmarks

        """TO PLAY WITH LANDMARKS INSTEAD OF OBJECTS"""
        static_things = objects.all_objects
        static_things = pins.poses_landmarks

        if __out: print "objects: " + repr(static_things)

        #Find Euclidean distance between each trajectory and the landmarks or objects :)
        objects_per_trajectory = ot.trajectory_object_dist(static_things, trajectory_poses)
        if __out: print objects_per_trajectory['7d638405-b2f8-55ce-b593-efa8e3f2ff2e']

    #**************************************************************#
    #          Apply QSR Lib to Objects and Trajectories           #
    #**************************************************************#
    #Dependant on Objects and Trajectories  
    __out = False
    if '2' in sections_dic:
        
        logger.info('2. Apply QSR Lib')
        qsr_dir, qsr_tag = qsr_setup(base_data_dir, qsr_params, date)
        qsr_out_file = os.path.join(qsr_dir + 'all_qsrs__' + qsr_tag + '.p')
        if __out: print qsr_out_file        


        try:
            spatial_relations = apply_qsr_lib(objects_per_trajectory, trajectory_poses, qsr_params, qsr_out_file)
        except NameError:
            print "2: 'Apply QSR Lib' needs Data. One of either: objects, trajectories, or parameters was not loaded correctly."
            if '1' not in sections_dic: print "Try re-running with arg = '1'\n"

    
    #**************************************************************#
    #             Generate Episodes from QSR Data                  #
    #**************************************************************#
    #Dependant on QSRs
    __out = False
    if '3' in sections_dic and '2' not in sections_dic:    
        logger.info('3. Loading QSR Data from Pickle File')
        qsr_dir, qsr_tag = qsr_setup(base_data_dir, qsr_params, date)
        qsr_out_file = os.path.join(qsr_dir + 'all_qsrs__' + qsr_tag + '.p')
        spatial_relations = pickle.load(open(qsr_out_file))
        if __out: print len(spatial_relations)

    if '3' in sections_dic:
        noise_threshold = 3
        qsr_dir, qsr_tag = qsr_setup(base_data_dir, qsr_params, date)
        epi_output_file = os.path.join(qsr_dir + 'episodes__' + qsr_tag + '.p')
        all_episodes = generate_episode_data(spatial_relations, epi_output_file, noise_threshold)
        if  __out: print "number of episode lists = " + repr(len(all_episodes))


    #**************************************************************#
    #            Activity Graphs/Code_book/Histograms              #
    #**************************************************************#
    #Dependant on Episodes
    __out = False
    if '4' in sections_dic and '3' not in sections_dic:
        
        qsr_dir, qsr_tag = qsr_setup(base_data_dir, qsr_params, date)
        all_episodes = pickle.load(open(os.path.join(qsr_dir + 'episodes__' + qsr_tag + '.p')))
        if  __out: print "number of episodes loaded = " + repr(len(all_episodes))

    if '4' in sections_dic:
        params, tag = AG_setup(my_data, date)
        activity_graph_dir = os.path.join(base_data_dir, 'AG_graphs/')
        check_dir(activity_graph_dir)
        if __out: print activity_graph_dir

        generate_graph_data(all_episodes, activity_graph_dir, params)
        if __out: print "Activity Graphs Done"

    #**************************************************************#
    #           Generate Feature Space from Histograms             #
    #**************************************************************#     
    __out = False
    if '5' in sections_dic:
        params, tag = AG_setup(my_data, date)
        activity_graph_dir = os.path.join(base_data_dir, 'AG_graphs/')
        if __out: print activity_graph_dir 
        generate_feature_space(activity_graph_dir, tag)


    """LEARN SOMETHING """ 
    __out = False
    if '6' in sections_dic and '5' not in sections_dic:       
        logger.info('6. Unsupervised Learning on Feature Space')
        learning_area = os.path.join(base_data_dir, 'relational_learner')
        check_dir(learning_area)
        if __out: print learning_area

        print "\nNOW LEARN...\n"

    print ""
    logger.info('Running rospy.spin()')
    rospy.spin()



    

    
    
