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
from soma_geospatial_store.geospatial_store import GeoSpatialStoreProxy

from qsrlib.qsrlib import QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace
from qsrlib_ros.qsrlib_ros_client import QSRlib_ROS_Client

import obtain_trajectories as ot
from novelTrajectories.traj_data_reader import *

from learningArea import Learning


#**************************************************************#
#      Create Activity Graphs For Each Trajectory Instance     #
#**************************************************************#

def generate_graph_data(episodes, data_dir, params):
    from Activity_Graph import Activity_Graph

    AG_out_file = os.path.join(data_dir + 'activity_graphs_' + tag + '.p')
    if __out: print AG_out_file

    cnt=0
    activity_graphs = {}

    for episodes_file in episodes:  

        uuid, start, end = episodes_file.split('__')        
        if __out: rospy.loginfo('Processing for graphlets: ' + episodes_file)

        episodes_dict = episodes[episodes_file]
        episodes_list = list(itertools.chain.from_iterable(episodes_dict.values()))

        activity_graphs[episodes_file] = Activity_Graph(episodes_list, params)
        activity_graphs[episodes_file].get_valid_graphlets()

        if __out and cnt == 0: graph_check(activity_graphs, episodes_file) #print details of one activity graph

        cnt+=1
        print cnt
      
    pickle.dump(activity_graphs, open(AG_out_file,'w')) 
    rospy.loginfo('4. Activity Graph Data Generated and saved to:\n' + AG_out_file) 
    return



def graph_check(gr, ep_file):
    """Prints to /tmp lots """
    gr[ep_file].graph2dot('/tmp/act_gr.dot', False)
    os.system('dot -Tpng /tmp/act_gr.dot -o /tmp/act_gr.png')
    print "graph: " + repr(ep_file)
    print gr[ep_file].graph

    gr2 = gr[ep_file].valid_graphlets
    for cnt_, i in enumerate(gr2[gr2.keys()[0]].values()):
        i.graph2dot('/tmp/graphlet.dot', False) 
        cmd = 'dot -Tpng /tmp/graphlet.dot -o /tmp/graphlet_%s.png' % cnt_
        os.system(cmd)



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

    rospy.loginfo('5. Generating codebook FINISHED')

    rospy.loginfo('5. Generating features')
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
    
    rospy.loginfo('Done')
    return feature_space



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
 
    run_options = {'1': "Get Object/Trajectories and Generate Graphs",
                   '2': "Unsupervised learning on feature space",
                   '7': "Test"}

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


    rospy.loginfo("0. Running ROI query from geospatial_store")
    gs = GeoSpatialStoreProxy('geospatial_store','soma')
    ms = GeoSpatialStoreProxy('message_store','soma')
    soma_map = 'uob_library'
    soma_config = 'uob_lib_conf'
    config_path = '/home/strands/STRANDS/config.ini'

    #*******************************************************************#
    #             Obtain ROI, Objects and Trajectories                  #
    #*******************************************************************#
    __out = False
    all_objects={}

    for roi in gs.roi_ids(soma_map, soma_config):
        if '1' not in sections_dic: 
            continue

        if roi != '12':
            continue

        if __out: print 'ROI: ', gs.type_of_roi(roi, soma_map, soma_config), roi
        geom = gs.geom_of_roi(roi, soma_map, soma_config)
        
        rospy.loginfo('1. Load trajectories in ROI')
        res = gs.objs_within_roi(geom, soma_map, soma_config)
        if res == None:
            print "No Objects in this Region"            
            continue

        objects_in_roi = {}
        for i in res:
            key = i['type'] +'_'+ i['soma_id']
            objects_in_roi[key] = ms.obj_coords(i['soma_id'], soma_map, soma_config)
            if __out: print key, objects_in_roi[key]

        if __out: print "Number of objects in region = " + repr(len(objects_in_roi))
        all_objects[roi] = objects_in_roi

        query = '''{"loc": { "$geoWithin": { "$geometry": 
        { "type" : "Polygon", "coordinates" : %s }}}}''' %geom['coordinates']
        q = ot.query_trajectories(query)
        trajectory_poses = q.trajs
        
        if len(trajectory_poses)==0:
            print "No Trajectories in this Region"            
            continue
        else:
            print "number of unique traj returned = " + repr(len(trajectory_poses))

        #objects_per_trajectory = ot.trajectory_object_dist(objects_in_roi, trajectory_poses)
   
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
        #keeper = Trajectory_QSR_Keeper(objects=objects_in_roi, \
        #                    trajectories=trajectory_poses, reader=reader)
        #keeper.save(base_data_dir)
        load_qsrs = 'all_qsrs_qtcb__0_01__False__True__03_03_2015.p'
        keeper= Trajectory_QSR_Keeper(reader=reader, load_from_file = load_qsrs, dir=base_data_dir) 

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
        activity_graph_dir = os.path.join(base_data_dir, 'AG_graphs/')
        if __out: print params, tag, activity_graph_dir

        generate_graph_data(ep.all_episodes, activity_graph_dir, params)
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
        __out = True
        rospy.loginfo('6. Learning on Feature Space')

        activity_graph_dir = os.path.join(base_data_dir, 'AG_graphs/')
        feature_space = pickle.load(open(os.path.join(activity_graph_dir, \
                        'feature_space_None_1_3_4__19_02_2015.p')))
        (cb, gb, X_source_U) = feature_space

        smartThing=Learning(f_space=X_source_U, c_book=cb, g_book=gb, vis=__out)
    
        smartThing.kmeans() #Can pass k, or auto selects min(penalty)
        smartThing.save(learning_area)
        smartThing.load(learning_area, 'smartThing.p')


    if '3' in sections_dic:  
        smartThing=Learning(load_from_file='smartThing.p', dir = learning_area)





    sys.exit(1)
    logger.info('Running rospy.spin()')
    rospy.spin()



    

    
    
