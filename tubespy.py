import numpy as np
from numpy import load

#######################################################################################################
                                            # CALUCLATE DEPTH MAP BIN
#######################################################################################################

def calculate_depth_map_bin(depth_map_filename, t_ready, t_deadline, vessel_draft):
    # Importar el mapa de profundidad
    depth_map = load(depth_map_filename)

    # Crear un mapa binario de profundidad según el borrador de la embarcación
    depth_map_bin = np.zeros([depth_map.shape[0], depth_map.shape[1]])
    for i in range(t_ready, t_deadline):
        for j in range(0, depth_map_bin.shape[1]):
            if abs(depth_map[i][j]) >= vessel_draft:
                depth_map_bin[i][j] = 1

    return depth_map_bin

######################################################################################################
                           # CALCULATE WINDOWS IN A CRITIC POINT
######################################################################################################
def calculate_window_entries_and_exits(depth_map_bin, t_ready, t_deadline, column):
    entry = []
    exit = []
    window = []

    for i in range(t_ready, t_deadline):
        if i == t_ready and depth_map_bin[i, column] == 1:
            entry.append(i)
        if depth_map_bin[i, column] == 0 and depth_map_bin[i + 1, column] == 1:
            entry.append(i + 1)
    for i in range(t_ready, t_deadline):
        if depth_map_bin[i, column] == 1 and depth_map_bin[i + 1, column] == 0:
            exit.append(i)
        if i == t_deadline - 1 and depth_map_bin[i, column] == 1:
            exit.append(i)

    for i in range(0, len(entry)):
        window.append([entry[i], exit[i]])

    return window

                                           
######################################################################################################
                          # TUBES FINDING FUNCTION (3 WWINDOW SIMPLIFICATION)
######################################################################################################
def tubes_finding (vessel_draft, t_ready, t_deadline, updown, critic_points, dist, speed_limits_inv):
    
    ##########################
    # CALCUATE BINARY MAP
    ##########################
    
    depth_map_filename = 'depth_map.npy'
    depth_map_bin = calculate_depth_map_bin(depth_map_filename, t_ready, t_deadline, vessel_draft)
    
    ################################################
    # CALCULATE CROSSING WINDOWS
    ################################################
    
    w_1 = calculate_window_entries_and_exits(depth_map_bin, t_ready, t_deadline, critic_points[0])
    w_2 = calculate_window_entries_and_exits(depth_map_bin, t_ready, t_deadline, critic_points[1])
    w_3 = calculate_window_entries_and_exits(depth_map_bin, t_ready, t_deadline, critic_points[2])    
  
    ################################################    
                    # PATH ROUTING       
    ###############################################
    if updown == 0:
        path_from_1_to_2 = []
        for i in range(0, len(w_1)):
            time_arrival_to_2 = w_1[i][0] + 10*(np.dot(dist[0:12], speed_limits_inv[0:critic_points[1]]))     
        
            for j in range(0, len(w_2)):
                if time_arrival_to_2 <= w_2[j][1]:
                    path_from_1_to_2.append([w_1[i],[max(w_2[j][0],time_arrival_to_2 ),w_2[j][1]]])   
        path_from_2_to_3 = []
        time_arrival_to_3_hist = [];  
        for i in range(0, len(path_from_1_to_2)):      
            time_arrival_to_3 = path_from_1_to_2[i][1][0] + 10*(np.dot(dist[critic_points[1]:24], speed_limits_inv[12:24]))  
            time_arrival_to_3_hist.append(time_arrival_to_3)
            for j in range(0, len(w_3)):
                if time_arrival_to_3 <= w_3[j][1]:
                    path_from_2_to_3.append([path_from_1_to_2[i][0], path_from_1_to_2[i][1], w_3[j]])
    else:
        path_from_3_to_2 = []
        time_arrival_to_2_hist = [];
        for i in range(0, len(w_3)):
            time_arrival_to_2 = w_3[i][0] + 10*(np.dot(dist[12:24], speed_limits_inv[12:24]))     
            time_arrival_to_2_hist.append(time_arrival_to_2)
            for j in range(0, len(w_2)):
                if time_arrival_to_2 <= w_2[j][1]:
                    path_from_3_to_2.append([w_3[i],[max(w_2[j][0],time_arrival_to_2 ),w_2[j][1]]])      
           
        path_from_2_to_1 = []
        time_arrival_to_1_hist = [];

        for i in range(0, len(path_from_3_to_2)):      
            time_arrival_to_1 = path_from_3_to_2[i][1][0] + 10*(np.dot(dist[0:12], speed_limits_inv[0:12]))  
            time_arrival_to_1_hist.append(time_arrival_to_1)
            for j in range(0, len(w_1)):
                if time_arrival_to_1 <= w_1[j][1]:
                    path_from_2_to_1.append([path_from_3_to_2[i][0], path_from_3_to_2[i][1], w_1[j]])
                        
    if updown == 0:
        tubes = path_from_2_to_3
    elif updown == 1:
        tubes = path_from_2_to_1
    
    return tubes


