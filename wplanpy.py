# -*- coding: utf-8 -*-
"""
Created on Thu Sep 14 16:59:18 2023

@author: jmnad
"""
import tubespy
from mip import Model, xsum, minimize, BINARY
import numpy as np


def planning(n_up, n_down, time_ready_up, time_ready_down, beam_up, beam_down, vmax_up, vmax_down, draft_up, draft_down, time_dead_up, time_dead_down, critic_points, dist, width, speed_limits, speed_limits_inv, waypoints):
    
    ###########################################################################
                                 # TUBE FINDING (encapsular esto)
    ###########################################################################                             
    
    # Tubes for vessels UP
    tubes_up = []
    n_tubes_up = []
    updown = 0;
    for i in range(0,n_up):
         tubes =  tubespy.tubes_finding (draft_up[i], time_ready_up[i], time_dead_up[i],updown, critic_points, dist, speed_limits_inv)
         tubes_up.append(tubes)
         n_tubes_up.append(len(tubes))
         
    # Tubes for vessels down
    tubes_down = []
    n_tubes_down = []
    updown = 1
    for i in range(0,n_down):
         tubes =  tubespy.tubes_finding (draft_down[i], time_ready_down[i], time_dead_down[i],updown, critic_points, dist, speed_limits_inv)   
         tubes_down.append(tubes)
         n_tubes_down.append(len(tubes))         
    
    ###########################################################################
                                 #OPTIMIZATION PROBLEM
    ###########################################################################
                             
    # Model Creation
    m = Model(sense=minimize,solver_name='GRB')
    
    # Time varibales
    t_u = [[m.add_var() for i in range(0,len(waypoints))]for j in range(0,n_up)]
    t_d = [[m.add_var() for i in range(0,len(waypoints))]for j in range(0,n_down)]
    
    #  Crossing variable
    c = [[m.add_var(var_type=BINARY) for i in range(0,len(waypoints))] for j in range(0, n_up * n_down)]
    
    # Route variables
    r_up   = [m.add_var(var_type=BINARY) for i in range(0,np.sum(n_tubes_up))]
    r_down = [m.add_var(var_type=BINARY) for i in range(0,np.sum(n_tubes_down))]
    
    # Cost function
    m.objective = minimize(xsum(t_u[i][len(waypoints)-1]-t_u[i][0]  for i in range(0,n_up)) 
                          +xsum(t_d[i][0]-t_d[i][len(waypoints)-1]  for i in range(0,n_down))
                          +xsum(t_u[i][0]-time_ready_up[i]  for i in range(0,n_up))
                          +xsum(t_d[i][len(waypoints)-1]-time_ready_down[i]  for i in range(0,n_down)))
    
    # Speed constraints
    for i in range(0,n_up):
        for j in range(0,len(waypoints)-1):
            m += min(vmax_up[i],speed_limits[j])*(t_u[i][j+1]-t_u[i][j])/10 >= dist[j]
    for i in range(0,n_down):
        for j in range(0,len(waypoints)-1):
            m += min(vmax_down[i],speed_limits[j])*(t_d[i][j]-t_d[i][j+1])/10 >= dist[j]        
       
    #Routes constraints
    M = 1000000;
    route_points = [0,12,24]
    for i in range(0,n_up):
        for j in range(0,n_tubes_up[i]):
            indice = int(np.sum(n_tubes_up[0:i]) + j)
            for p in range(0,len(route_points)):
                m += t_u[i][route_points[p]] + (1-r_up[indice])*M >= tubes_up[i][j][p][0] 
                m += t_u[i][route_points[p]] <= tubes_up[i][j][p][1] + (1-r_up[indice])*M                     
    for i in range(0, len(n_tubes_up)):
        m += xsum(r_up[j] for j in range(sum(n_tubes_up[0:i]),sum(n_tubes_up[0:i+1]))) == 1
        
    route_points = [24,12,0]
    for i in range(0,n_down):
        for j in range(0,n_tubes_down[i]):
            indice = int(np.sum(n_tubes_down[0:i]) + j)
            for p in range(0,len(route_points)):
                m += t_d[i][route_points[p]] + (1-r_down[indice])*M >= tubes_down[i][j][p][0] 
                m += t_d[i][route_points[p]] <= tubes_down[i][j][p][1] + (1-r_down[indice])*M                    
    for i in range(0, len(n_tubes_down)):
        m += xsum(r_down[j] for j in range(sum(n_tubes_down[0:i]),sum(n_tubes_down[0:i+1]))) == 1
                    
    # Crossing constraints
    for i in range(0,n_up):
        for j in range(0,n_down):
            index = n_down*i + j
            for p in range(0,len(waypoints)):
                m += t_u[i][p] <= t_d[j][p] + M * xsum(c[index][l] for l in range(0, p))
                m += t_d[j][p] <= t_u[i][p] + M * (1-xsum(c[index][l] for l in range(0, p)))
    for i in range(0,n_up):
        for j in range(0,n_down):
            index = n_down*i + j
            m += xsum(c[index][l] for l in range(0,len(waypoints))) <= 1
    
    # Width constraints    
    for i in range(0,n_up):
        for j in range(0,n_down):
            for k in range(0,len(width)):
                index = n_down*i + j
                if beam_up[i] + beam_down[j] >=  width[k]:
                    m += c[index][k] == 0      
    # Optimization
    m.optimize()
    
    # Solution
    plan = np.zeros((n_up+n_down,len(waypoints)))
    for i in range(0,n_up+n_down):
        for j in range(0,len(waypoints)):
            plan[i][j] = m.vars[len(waypoints)*(i)+j].x
    return plan, tubes_up, tubes_down, n_tubes_up, n_tubes_down