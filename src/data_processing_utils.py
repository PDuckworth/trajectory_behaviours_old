#!/usr/bin/env python

"""data_processing_utils.py"""

__author__      = "Paul Duckworth"
__copyright__   = "Copyright 2014, University of Leeds"

import os, sys
import numpy as np


#**************************************************************#
#     Compute Episode Representation of the Spatial Relations  #
#**************************************************************#


def compute_episodes(spatial_relations, cnt, __out):
    #print "# loaded tracjectories: " + repr(len(spatial_relations))

    episodes  = {}
    obj_relation_data = {}
    frames = spatial_relations.keys()
    frames.sort()
    for frame in frames:
        for obj_pair in spatial_relations[frame]:
            if obj_pair not in obj_relation_data:
                obj_relation_data[obj_pair] = []
            obj_relation_data[obj_pair].append((frame, spatial_relations[frame][obj_pair]))

    for uuid, obj in obj_relation_data:
        obj_0_id = uuid
        obj_0_type = 'traj'

        obj_1_id = obj
        obj_1_type = obj.split('_')[0]

        obj_pair=(uuid, obj)
        episodes[(obj_0_id, obj_0_type, obj_1_id, obj_1_type)] = []
        epi_start = obj_relation_data[obj_pair][0][0]
        epi_end   = obj_relation_data[obj_pair][0][0]
        epi_value = obj_relation_data[obj_pair][0][1]
        for (frame, relation) in obj_relation_data[obj_pair]:
            if relation == epi_value:
                epi_end = frame
            else:
                episodes[(obj_0_id, obj_0_type, obj_1_id, obj_1_type)].append((obj_0_id, obj_0_type,\
                                                                               obj_1_id, obj_1_type,\
                                                                               epi_value, epi_start, epi_end))
                epi_start = epi_end = frame
                epi_value = relation
        else:
            episodes[(obj_0_id, obj_0_type, obj_1_id, obj_1_type)].append((obj_0_id, obj_0_type,\
                                                                           obj_1_id, obj_1_type,\
                                                                           epi_value, epi_start, epi_end))           
    episodes_key = '__'.join([uuid, repr(min(frames)), repr(max(frames))])
    if __out: print repr(cnt) + "    " + repr(episodes_key)
    return episodes_key, episodes    


def filter_intervals(intv_list_dup, noise_threshold):
    intv_list = intv_list_dup[:]
    intv_list 
    filtered_intv_list = []
    next_relation_threshold = noise_threshold
    
    newf = None
    obj1_id   = 0
    obj1_type = 1
    obj2_id   = 2
    obj2_type = 3
    rel       = 4
    start     = 5
    e         = 6
    for f in intv_list:
        #PD: This doesn't check whether the first f is below the threshold
        if newf is None:
            newf = list(f[:])
        elif newf[rel] == f[rel]:
            # both intervals have same relations, just merge
            newf[e] = f[e]
        else:
            # intervals have different relations
            if f[e] - f[start] < next_relation_threshold: 
                # interval of relation is too small, so merge with previous one
                newf[e] = f[e]
            else:
                # This interval is reasonably big, so don't merge it.
                filtered_intv_list.append(tuple(newf))
                newf = list(f[:])
    else:            
        # Reached the end of list. so Finish by adding the last interval (newf not f)
        filtered_intv_list.append(tuple(newf))
    return filtered_intv_list    


if __name__ == '__main__':
    base_data_dir = '/home/strands/STRANDS/'
    qsr_dir = os.path.join(base_data_dir, 'qsr_dump/')
    spatial_relations_1 = pickle.load(open(os.path.join(qsr_dir + 'single_traj.p')))

    """THEN CREATE EPISODES  """ 
    episodes = compute_episodes(spatial_relations_1)  
    print episodes  

