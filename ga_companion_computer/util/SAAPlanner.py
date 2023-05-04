# -*- coding: utf-8 -*-
"""
Created on Wed Jul  8 14:07:56 2020

@author: Suryansh Aryan
@Organization: General Aeronautics Pvt Ltd
"""

import math
import numpy as np
import util.VectorMath as vmath
import util.SAADataHandling as datahand
import matplotlib.pyplot as plt
import logging
import time
import sys
np.set_printoptions(threshold=sys.maxsize)

class Planner():
    '''Important note for this class: 
    Inputs : Obstacle map in inertial origin centered map and Waypoints in [lat long] format.
    Outputs : A,B,C point position in inertial frame origin centered ma at that time instance (def Guided_waypoint)
    Note : Outputs would be calculated at each timestep while Guided Navigaiton is triggered, so values of A,B,C would change each timestep.
    '''

    def __init__(self, safety_offset=[0,4]):
        # home_pos ---> Home position given by Ardupilot (lat, long)
        # waypoints ---> array of all waypoint 2D position in order
        # safety_offset ---> Index 0 is offset for point A  | Index 1 is offset for path planning end position so indirectly offset on point C

        self.px = 0
        self.py = 0

        self.vec = vmath.vector()
        self.frame = datahand.DataPostProcessor()
        self.pathpoints = np.array([])  # Inertial frame origin centered
        self.safety_offset = safety_offset
        self.w_vector = np.array([])   # Current waypoint being targeted
        self.w_prev = np.array([])     # Stores previous target waypoint, default value for start is home position
        self.w_current = np.array([])  # Stores current target waypoint as third variable to assign it to w_prev variable
        self.wp_home_center = np.array([len(self.w_vector), 2])
        self.w_prev_home_center = np.array([])
        self.id = 0
        self.home_pos = np.array([])
        self.c_wp_vec = np.array([])  # Gives instantaneous waypoint unit vector for the mission
        self.plan_command = 0  # Trigger that indicates planning is done now maneuvering is allowed

        #self.make_wp_origin_center()

    def make_wp_origin_center(self):
        if self.home_pos.size != 0 and self.w_prev.size == 0:
            self.w_prev = self.home_pos
        
        if self.w_current.size == 0 and self.w_vector.size != 0:
            self.w_current = self.w_vector
        
        if not np.array_equal(self.w_vector, self.w_current) and self.w_current.size != 0:
            self.w_prev = self.w_current
            self.w_current = self.w_vector

        #### Till here w_prev, w_current and w_vector are in (lat, long) ###

        if self.w_vector.size != 0 and self.w_prev.size != 0:

            world_x = 111000*(self.w_prev[0] - self.home_pos[0])
            world_y = 87360*(self.w_prev[1] - self.home_pos[1])
            self.w_prev_home_center = np.array([world_x, world_y])

            if self.w_vector.shape == (2,):
                world_x = 111000*(self.w_vector[0] - self.home_pos[0])
                world_y = 87360*(self.w_vector[1] - self.home_pos[1])
                self.wp_home_center = np.array([world_x, world_y]).T.reshape([1,2])
            else:    
                world_x = 111000*(self.w_vector[:,0] - self.home_pos[0])
                world_y = 87360*(self.w_vector[:,1] - self.home_pos[1])
                self.wp_home_center = np.array([world_x, world_y]).T.reshape([len(self.w_vector), 2])

            self.c_wp_vec = (self.wp_home_center[0] - self.w_prev_home_center)/np.linalg.norm(self.wp_home_center[0] - self.w_prev_home_center)

    def Pathplanning(self, obs, plot=False):
        # obs --> Obstacle map in inertial frame drone centered
        # self.pathpoints is in origin centered
        print('Entered path planning')    

        # It everytime looks upon obstacle posiiton and updates the path
        #obs_nearby_drone_cent = np.array([i for i in obs if self.vec.mag2d(i)<=25])
        obs_nearby_drone_cent = obs

        if obs_nearby_drone_cent.size != 0 and self.c_wp_vec != []:
            # Update the frame object's position variable using Planner position variable so that it can be used for reference frame transformations
            self.frame.px = self.px
            self.frame.py = self.py

            # obs_nearby_origin_cent = self.frame.convert_rel_to_inertial_avoid(obs_nearby_drone_cent)
            # O = obs_nearby_origin_cent
            # s = np.array([self.px, self.py])
            # e = (np.max(np.dot(O, self.c_wp_vec)))*self.c_wp_vec 

            s = np.array([self.px, self.py])
            e = self.frame.convert_rel_to_inertial_avoid((np.max(np.dot(obs_nearby_drone_cent, self.c_wp_vec)))*self.c_wp_vec)
            obs_nearby_origin_cent = self.frame.convert_rel_to_inertial_avoid(obs_nearby_drone_cent)
            O = obs_nearby_origin_cent

            #mean_dist = 5

            A = []
            for i in range(len(O)):
                if np.linalg.norm(s-O[i]) < 3*np.linalg.norm(s-e) and np.linalg.norm(s-O[i])<30:
                    A.append(O[i,:])

            A = np.array(A)
            A_drone_cent = self.frame.convert_inertial_to_rel_avoid(A)

            e = self.frame.convert_rel_to_inertial_avoid((np.max(np.dot(A_drone_cent, self.c_wp_vec))+self.safety_offset[1])*self.c_wp_vec).reshape(2,)  # End position 5 meters away from farthest obstacle in wp_vector direction

            #print(f'e --> {e}')
            #print('-----------------------')
            #print(f'Obstacle in path planning : {A}')

            # y = np.cross(np.array([(s-e)[0], (s-e)[1], 0]), np.array([0,0,1]))
            # y = np.array([y[0], y[1]])
            # mean_dist = np.min([i for i in np.dot(A,y) if abs(i) < 5])

            ### Planning Started #####
            thres_dist = 0.1
            G = 16  # 16
            E = 14/len(A[:,1])**3  #18/len(A[:,1])**3
            R = 2
            dt = 0.05
            d_obs = np.array([])
            dist_obs = np.array([])
            F_obs_mag = []
            F_obstacle = []
            Pos = []
            self.pathpoints = np.array([])

            # Goal potential field
            theta = np.arctan2((s[1]-e[1]), (s[0]-e[0]))
            d_goal = np.linalg.norm(s-e)*np.array([np.cos(theta), np.sin(theta)])    # displacement between goal and drone
            dist_goal = np.linalg.norm(d_goal)

            # Obstacle potential field         Made it such that weak for far obstacles
            # but strong for nearby obstacles   F_obs inversely proportional to dist^4
            d_obs = s - A
            dist_obs = np.array([np.linalg.norm(i) for i in d_obs])

            d = d_goal
            dist = dist_goal

            v = [0,0]

            # plt.figure()
            # plt.plot(s[0],s[1],'oy')
            # plt.plot(e[0],e[1],'og')

            ############### Multiple Obstacle in Plot #####################
            # for i in range(len(A[:,1])):
            #     plt.plot(A[i,0],A[i,1],'*b')

            t = 1
            while dist > thres_dist: 
                F_obstacle = []  
                F_obs_mag = []
                if dist_goal > 2:
                    F_goal_mag = G/(dist_goal)**2
                else:
                    F_goal_mag =  50/(dist_goal)**4


                F_goal = -F_goal_mag*d_goal     # Yes this is a vector

            ################## Multiple obstacles logic ###################
                for i in range(len(A[:,1])):
                    if dist_obs[i] > 1: #5  
                        if dist_obs[i] > 4*dist_goal:
                            F_obs_mag.append(0)
                        else:
                            F_obs_mag.append(8*E/(dist_obs[i] - R)**2)  # 8*E/(dist_obs[i] - R)**2
                    else:
                        F_obs_mag.append(0.01*E/(dist_obs[i])**4)  #3*E/(dist_obs[i])**4

                    F_obstacle.append(F_obs_mag[i]*d_obs[i,:])         # Again a vector

                F_obstacle = np.array(F_obstacle)
                F_obs = np.array([sum(F_obstacle[:,0]), sum(F_obstacle[:,1])])

                
                F_tot = F_goal + F_obs          # Total force using vector addition
                
                if np.linalg.norm(F_tot) > 2:
                    F_tot = 2*F_tot/np.linalg.norm(F_tot)
                
                v = v + F_tot*dt
                if np.linalg.norm(v) > 0.5:
                    v = 0.5*v/np.linalg.norm(v)
                
                d = d + v*dt
                dist = np.linalg.norm(d)
                
                # Update d_goal and d_obs           
                # d_goal is actually d and correspondingly its magnitude
                d_goal = d
                dist_goal = dist
                
                ######## Multiple Obstacle ##########
                d_obs = d - (A - e)
                dist_obs = [np.linalg.norm(i) for i in d_obs]
                #####################################
                
                # Update drone position wrt world frame for plotting
                S = e + d_goal
                Pos.append(S)
                
                t = t + 1
    
            self.pathpoints = np.array([Pos]).reshape([-1,2])
            self.Guided_waypoint()

            # if plot == True and self.pathpoints.size != 0:
            #     plt.plot(self.pathpoints[:,0], self.pathpoints[:,1], '.r')
            #     #annotation('arrow', [0,v(1)/norm(v,2)], [0,v(2)/norm(v,2)])
            #     #plt.xlim([-10,10])
            #     # plt.ylim([-30,30])
            #     plt.show()
        



    def Guided_waypoint(self):
        # It just looks the path made by the Pathplanning() and doesnt care about obstacle positions
        y_unit_vec = np.cross(np.array([self.c_wp_vec[0],self.c_wp_vec[1],0]), np.array([0,0,1]))
        y_unit_vec = np.array([y_unit_vec[0], y_unit_vec[1]])
        pathpoints_drone_cent = self.frame.convert_inertial_to_rel_avoid(self.pathpoints)
        max_path_deviate_arg = np.argmax(abs(np.dot(pathpoints_drone_cent, y_unit_vec)))   # abs used because np.dot can give neg values and we want absolute farthest path point

        # All (A,B,C) positions are in origin centered
        self.A_pos_origin_cent = np.array([self.px, self.py]) + (np.dot(y_unit_vec, pathpoints_drone_cent[max_path_deviate_arg])+ np.sign(np.dot(y_unit_vec, pathpoints_drone_cent[max_path_deviate_arg]))*self.safety_offset[0])*y_unit_vec   # Vector to first Guided position wrt y unit vector --> array of size 2
        self.B_pos_origin_cent = self.A_pos_origin_cent + np.dot((self.pathpoints[-1]-self.A_pos_origin_cent), self.c_wp_vec)*self.c_wp_vec  # Vector to second Guided position wrt y unit vector --> array of size 2
        B_pos_drone_cent = self.frame.convert_inertial_to_rel_avoid(self.B_pos_origin_cent)
        self.C_pos_origin_cent = self.B_pos_origin_cent - np.dot(B_pos_drone_cent, y_unit_vec)*y_unit_vec  # Vector to third Guided position wrt y unit vector --> array of size 2
        print(f'A pos : {self.A_pos_origin_cent}  B pos : {self.B_pos_origin_cent}   C pos : {self.C_pos_origin_cent}')
        print(np.dot(y_unit_vec, self.pathpoints[max_path_deviate_arg]))
        self.plan_command = 1


