#!/usr/bin/env python

"""compute_spatial_relations.py: Computes spatial relations."""

__author__      = "Paul Duckworth"
__copyright__   = "Copyright 2015, University of Leeds"

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
from human_trajectory.msg import Trajectory, Trajectories
from soma_trajectory.srv import TrajectoryQuery, TrajectoryQueryRequest, TrajectoryQueryResponse
from soma_geospatial_store.geospatial_store import *

import imports.obtain_trajectories as ot
from imports.graphs_handler import *
from novelTrajectories.traj_data_reader import *
from imports.learningArea import Learning




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




if __name__ == "__main__":
    global __out
    rospy.init_node("trajectory_learner")

    parser = argparse.ArgumentParser()
    parser.add_argument("sections", help="chose what section of code to run", type=str)
    args = parser.parse_args()
 
    run_options = {'1': "Get Object/Trajectories and Generate Graphs",
                   '2': "Unsupervised learning on feature space",
                   '3': "Test"}

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
    learning_area = os.path.join(base_data_dir, 'learning/')
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
    config_path = '/home/strands/STRANDS/config.ini'


    soma_map = 'uob_library'
    soma_config = 'uob_lib_conf'
    gs = GeoSpatialStoreProxy('geospatial_store','soma')
    msg = GeoSpatialStoreProxy('message_store', 'soma')
    rospy.loginfo("0. Running ROI query from geospatial_store")   

    two_proxies = TwoProxies(gs, msg, soma_map, soma_config)
    
    #*******************************************************************#
    #             Obtain ROI, Objects and Trajectories                  #
    #*******************************************************************#
    __out = False
    for roi in gs.roi_ids(soma_map, soma_config):
        if '1' not in sections_dic: 
            continue
        if roi != '12':
            continue

        if __out: print 'ROI: ', gs.type_of_roi(roi, soma_map, soma_config), roi
        objects = two_proxies.roi_objects(roi)

        geom = two_proxies.gs.geom_of_roi(str(roi), soma_map, soma_config)

        if __out: print "  Number of objects in region = " + repr(len(objects))
        if __out: print "geometry of region= ", geom

        query = '''{"loc": { "$geoWithin": { "$geometry": 
        { "type" : "Polygon", "coordinates" : %s }}}}''' %geom['coordinates']
        
        

        q = ot.query_trajectories(query)
        trajectory_poses = q.trajs   #In xyz map coordinates
        
        if len(trajectory_poses)==0:
            print "No Trajectories in this Region"            
            continue
        else:
            print "number of unique traj returned = " + repr(len(trajectory_poses))

        #objects_per_trajectory = ot.trajectory_object_dist(objects, trajectory_poses)
   
        #LandMarks instead of Objects - need to select per ROI:
        #all_poses = list(itertools.chain.from_iterable(trajectory_poses.trajs.values()))
        #if __out: print "number of poses in total = " +repr(len(all_poses))
        #landmarks = ot.select_landmark_poses(all_poses)
        #pins = ot.Landmarks(landmarks)
        #if __out: print "landmark poses = " + repr(landmarks)
        #if __out: print pins.poses_landmarks

        #"""TO PLAY WITH LANDMARKS INSTEAD OF OBJECTS"""
        #object_poses = objects.all_objects
        #object_poses = pins.poses_landmarks



    #**************************************************************#
    #          Apply QSR Lib to Objects and Trajectories           #
    #**************************************************************#
    #Dependant on trajectories and objects
        __out = False
        rospy.loginfo('2. Apply QSR Lib')
        if __out: raw_input("Press enter to continue")

        reader = Trajectory_Data_Reader(config_filename = config_path)
        keeper = Trajectory_QSR_Keeper(objects=objects, \
                            trajectories=trajectory_poses, reader=reader)
        keeper.save(base_data_dir)
        load_qsrs = 'all_qsrs_qtcb__0_01__False__True__03_03_2015.p'
        #keeper= Trajectory_QSR_Keeper(reader=reader, load_from_file = load_qsrs, dir=base_data_dir) 

        #print keeper.reader.spatial_relations['7d638405-b2f8-55ce-b593-efa8e3f2ff2e'].trace[1].qsrs['Printer (photocopier)_5,trajectory'].qsr

    
    #**************************************************************#
    #             Generate Episodes from QSR Data                  #
    #**************************************************************#
    #Dependant on QSRs 
        __out = False
        rospy.loginfo('3. Generating Episodes')
        if __out: raw_input("Press enter to continue")

        ep = Episodes(reader=keeper.reader)
        ep.get_episodes(noise_thres=3, out=__out)
        ep.save(base_data_dir)
        if __out: print "episode test: " + repr(ep.all_episodes['5c02e156-493d-55bc-ad21-a4be1d9f95aa__1__22'])

    #**************************************************************#
    #            Activity Graphs/Code_book/Histograms              #
    #**************************************************************#
    #Dependant on Episodes
        __out = False
        rospy.loginfo('4. Generating Activity Graphs')
        if __out: raw_input("Press enter to continue")

        params, tag = AG_setup(my_data, date)
        print params
        print tag

        activity_graph_dir = os.path.join(base_data_dir, 'AG_graphs/')
        if __out: print params, tag, activity_graph_dir

        generate_graph_data(ep.all_episodes, activity_graph_dir, params, tag)
        if __out: print "Activity Graphs Done"


    #**************************************************************#
    #           Generate Feature Space from Histograms             #
    #**************************************************************#     
        rospy.loginfo('5. Generating Feature Space')
        if __out: raw_input("Press enter to continue")
        feature_space = generate_feature_space(activity_graph_dir, tag)
            
        
    #**************************************************************#
    #                    Learn a Clustering model                  #
    #**************************************************************#
    if '2' in sections_dic:   
        __out = False
        rospy.loginfo('6. Learning on Feature Space')

        activity_graph_dir = os.path.join(base_data_dir, 'AG_graphs/')
        feature_space = pickle.load(open(os.path.join(activity_graph_dir, \
                        'feature_space_None_1_3_4__19_02_2015.p')))
        (cb, gb, X_source_U) = feature_space

        smartThing=Learning(f_space=X_source_U, c_book=cb, g_book=gb, vis=__out)
    
        smartThing.kmeans(k=2) #Can pass k, or auto selects min(penalty)
        smartThing.save(learning_area)

    if '3' in sections_dic:
        file_ = os.path.join(learning_area, 'smartThing.p')
        print file_
        smartThing = Learning(load_from_file=file_)
        print dir(smartThing)
        print smartThing.code_book

    sys.exit(1)
    logger.info('Running rospy.spin()')
    rospy.spin()



    

    
    
