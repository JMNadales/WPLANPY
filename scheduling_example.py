# -*- coding: utf-8 -*-
"""
Created on Thu Sep 14 16:49:10 2023

@author: jmnad
"""


import wplanpy
import pymongo
import matplotlib.pyplot as plt


###############################################################################
                            # CONNECTION TO MONGODB
###############################################################################
# Connection uri
mongo_uri = "YOU DATA BASE URI"

# Connect to DB
client = pymongo.MongoClient(mongo_uri)
# Access DB
db = client."YOUR DATA BASE"

###############################################################################
                            # READ WATERWAY DATA
###############################################################################
# Select waterway data collection
collection_waterway = db.waterway_data
# Read document in waterway data collection
documents = collection_waterway.find()
for document in documents:
    waypoints    = document['waypoints']
    dist         = document['dist']
    speed_limits = document['speed_limits']
    width        = document['width']
speed_limits_inv = [1/x for x in speed_limits]

###############################################################################
                            # READ VESSELS DATA
###############################################################################
# Select waterway data collection
collection_vessels_up   = db."YOUR ORIGIN PORT"
collection_vessels_down = db."YOUR DESTINY PORT"

# Vessels beams
beam_up   = []
beam_down = []

# Vessels maximum speed
vmax_up   = []
vmax_down = []

# Vessels Draft
draft_up   = []
draft_down = []

# ETA
time_ready_up   = []
time_ready_down = []

# ETD
time_dead_up   = []
time_dead_down = []
DL = 180

documents_up = collection_vessels_up.find()
documents_down = collection_vessels_down.find()
n_up = collection_vessels_down.count_documents({})
n_down = collection_vessels_down.count_documents({})

for document in documents_up:
    beam_up.append(document['beam'])
    vmax_up.append(document['max_speed'])
    draft_up.append(document['draft'])
    time_ready_up.append(int(document['ETA']))  
time_dead_up = [x + DL for x in time_ready_up]

for document in documents_down:
    beam_down.append(document['beam'])
    vmax_down.append(document['max_speed'])
    draft_down.append(document['draft'])
    time_ready_down.append(int(document['ETA']))  
time_dead_down = [x + DL for x in time_ready_down]

# Select critic points where to analyse depth
critic_points = [0, 491, 870]

#####################################################################################################
                                        # TEST
#####################################################################################################                                        
[plan, tubes_up, tubes_down, n_tubes_up, n_tubes_down] = wplanpy.planning(n_up, n_down, time_ready_up, time_ready_down, beam_up, beam_down, vmax_up, vmax_down,draft_up, draft_down, time_dead_up, time_dead_down, critic_points,dist, width, speed_limits, speed_limits_inv, waypoints)


plt.rc('xtick', labelsize=16) 
plt.rc('ytick', labelsize=16) 


plt.figure(figsize=(10, 6), tight_layout=True, facecolor='white')
plt.rcParams['font.size'] = 20
plt.rcParams["font.family"] = "serif"
for i in range(0, n_up + n_down):
    plt.plot(waypoints, plan[i, 0:len(waypoints)], 'o--', linewidth=1)
plt.gca().set_facecolor('white')
plt.grid(True)
plt.xlabel('Position (km/10)', fontsize=16)
plt.ylabel('Time (h/10)', fontsize=16)
plt.show()
