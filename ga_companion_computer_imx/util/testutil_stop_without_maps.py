#!/home/dhruv/.local/lib/python3.8
# -*- coding: utf-8 -*-
"""
Created on Wed Jul  8 14:07:56 2020

@author: Sachchit Vekaria
@Organization: General Aeronautics Pvt Ltd
"""
#this has sense and stop code using a lidar

# import necessary modules
import pymap3d as pm
from os import O_DSYNC
from pickle import NONE
import time
from unittest import FunctionTestCase
from util.gacommonutil import CompanionComputer, mavutil, ScheduleTask, mavwp
import threading
import logging
from util.dijkstra import Dijkstra

# # # Another module
import math
import numpy as np
import matplotlib.pyplot as plt
import util.VectorMath as vmath
import util.SAADriver as driver
import util.SAADataHandling as estimation
import util.SAAController as control
import util.SAAPlanner as planner
import numpy as np
import sys
np.set_printoptions(threshold=sys.maxsize)


class TestCompanionComputer(CompanionComputer):
    def __init__(self, sitlType, sitlPort):
        # Initialize super class
        super().__init__(sitlType, sitlPort)
        
        # Threading Lock for TestCompanionComputer Class
        self.lock = threading.Lock()
        self.handleRecievedMsgThread = None

        #Initialise SITL driver
        self.lidar = driver.SensorDriver('SITL')

        #Connect to the listener - ensure the listener is running in background!!
        self.lidar.connect_and_fetch()
        self.guided_target=[]

        #Front sensor
        #self.front_sensor = estimation.Sensor(1,1*math.pi/180,12,0.01,0)
        #SITL
        self.front_sensor = estimation.Sensor(1,0.03098,40,1,-0.976)  
        self.marker=0
        self.checker=False
        #Initialise pre processor
        self.coordinate_transform = estimation.DataPreProcessor()

        #initialise navigation controller
        self.navigation_controller = control.ObstacleAvoidance(max_obs=20+3)

        self.navigation_map = estimation.DataPostProcessor()
        #assumes 750 m field on either side
        self.field_x=1500
        self.field_y=1500
        self.checked=1


        self.matmap=np.array([[0]*self.field_x]*self.field_y)
        np.reshape(self.matmap,(self.field_x,self.field_y))
        self.width_fetcher=100
        self.length_fetcher=1498
        self.relmatmap=np.array([[0]*self.width_fetcher]*self.length_fetcher)



        #Initialize vector class of Vectormath
        self.vec = vmath.vector()

        ##########################################  Find out how to take out home and waypoint location via Mavlink

        #Initialize Planner class
        self.plan = planner.Planner()

        # Home posiiton
        self.home = np.array([])

        # Waypoint position 
        self.wp = np.array([])
        self.avoidance_mode=0
        #Brake
        self.brake = 0
        self.alreadybraked = 0
        self.initvar=1

        #Initiatialising Position Predictor Vectors
        self.prev_px = 0
        self.prev_py = 0
        self.px = 0
        self.py = 0
        self.engaging_distance=25
        self.desired_ang=10
        self.count_dum=0
        
        
        self.count_iter=0
        self.count_mat=0
        self.previous_waypoint = 0
        self.i_previous_waypoint = []
        self.changed_from_auto_to_guided=0
        self.obstacle_stopping_angle=15 # in degrees


        # Terminate trigger
        self.terminate = 0    
        
        # Pilot overriding
        self.overriding = 1    # Initially pilot control
        self.dist_temp=0

        self.t2 = 0
        self.res=0
        self.obs_min_in_direct=100
        self.reached_A=self.reached_B=self.reached_C=self.reached_D=self.reaching_A=self.reaching_B=self.reaching_C=self.reaching_D=False
        self.c_point_index=self.p_point_index=0
        self.safety_margin=10
        self.g = 0
        self.v = 0
        self.t = 0
        self.ti = 0
        self.tt1 = 0
        self.simulation=-1 # 1 for normal case and -1 for simulation
        #todo

        self.A_reached = 0
        self.B_reached = 0
        self.C_reached = 0

        self.obstacle_map_optim = np.array([[]])
        self.move_direc=[]
        self.direction_decided=0
        self.current_waypoint = 0
        self.next_waypoint = 0
        self.waypoint_count = 0
        self.wpvec=[]
        self.nextwaypoint = 0
        self.waypoints = []
        self.i_waypoints =  []
        self.i_current_waypoint = []
        self.i_next_waypoint = []
        self.sensor_trig = 0 #Use this as an obstacle avoidance triggerer
        self.mat_list=[]
        self.mat_count=0
        self.boundary_points=np.array([[-2,2],[-2,-302],[10,-302],[10,2]])
        self.dummy_counteer=1
        

    def init(self):
        super().init()

        # set data stream rate
        self.set_data_stream()# mavlink messages  sent for getting position
        
        # start our recieving message handling loop
        self.handleRecievedMsgThread = threading.Thread(target=self.handle_recieved_message)
        self.handleRecievedMsgThread.start()

        ### Starting reading threads as they are while loops ###
        #t1 = threading.Thread(target=self.lidar.give_scan_values)
        #t1.start()
        #t2 = threading.Thread(target=self.####I need to compute thislidar.read_fast)
        #t2.start()
        
        
        # set data stream rate
        #self.set_data_stream()
        
        # start our recieving message handling loop
        #self.handleRecievedMsgThread = threading.Thread(target=self.handle_recieved_message)
        #self.handleRecievedMsgThread.start()

        #Scheduled the threads
        print("Executing all threads")
        print("Fetchin all the waypoints")
        self.Obtain_Waypoints() #obtains details of waypoint at start of misison
        
        #self.scheduledTaskList.append(ScheduleTask(0.02, self.lidar.update_sitl_sensor))  # 0.02
        #self.scheduledTaskList.append(ScheduleTask(0.000000000000000001, self.lidar.give_scan_values))
        #self.give_scan_values()
        # self.scheduledTaskList.append(ScheduleTask(0.00002,self.lidar.update_rplidar))
        t2 = threading.Thread(target=self.lidar.update_sitl_sensor)#/*/* returns a list in self.raw_data
        #stores data to self.lidar.raw_data to 64 elements ordered.
        t2.start() #/*/*check
       
       
        #self.scheduledTaskList.append(ScheduleTask(0.1,self.check_sensor_Raw))
       
        t3 = threading.Thread(target=self.Obtain_Current_Waypoint)
        t3.start()

        print("All threads started!")
        print("Starting The Front sensor sensor in 10 seconds")
        time.sleep(10)
        #self.check_sensor_Raw()

        

           
#   takes in direction vector and a map in global coordinates to give minimum distance to obstacle in that sector, returns true if obstacle is present                        
        while True:
            self.update_mission() #here all the logic to execute mission is written

         
    def print_waypoint(self):
        while(True):
            time.sleep(20)
            print('current wp is', self.c_point_index)
            if(self.dummy_counteer==1):
                self.jump_wayPoint(4)
                self.dummy_counteer=0
            time.sleep(1)

    def obstacle_in_direction(self,direc_vec,global_map,des_angle,des_distance):
        obs_in_direc_marker=0
        obs_min_in_direct=100 #returns 100 if there is nothing within des_distance else gives min_distance
        for i in range(np.size(global_map,axis=0)):
                #Compute vector
            obstacle_vector = [global_map[i,0]-self.px,global_map[i,1]-self.py]

            obstacle_vector_magnitude=self.vec.mag2d(obstacle_vector)

            obstacle_mag_in_direc=(np.dot(direc_vec,obstacle_vector)/(self.vec.mag2d(direc_vec)))
                
            if((obstacle_vector_magnitude>=0.5 and obstacle_vector_magnitude<=des_distance)and obstacle_mag_in_direc>0):                    
                obstacle_angle = round(math.acos(np.dot(direc_vec,obstacle_vector)/(self.vec.mag2d(direc_vec)*self.vec.mag2d(obstacle_vector)))*180/math.pi,2)
                if abs(obstacle_angle)<des_angle :
                    obs_in_direc_marker=1
                    if((obstacle_mag_in_direc)<obs_min_in_direct):
                        obs_min_in_direct=obstacle_mag_in_direc                  
                
        if(obs_in_direc_marker==1):
            print('found obstacle in direction',direc_vec,'in global map',global_map,'from position',round(self.px),round(self.py))
            return True,obs_min_in_direct

            
        else:
            return False,obs_min_in_direct


    def basic_stop(self):#triggers avoidance
        """
        Basic Stopping Class
        """
        if self.relativeAlt>1:
            bool_temp,self.dist_temp=self.obstacle_in_direction(self.navigation_controller.pos_vector,self.np_mat_to_list,self.desired_ang,self.engaging_distance)
            if(bool_temp): #direction vector and angle
                #todo add waypoint vector above

                self.brake = 1
                print('hey detected obstacle_braking')
                                
            else:
                print("Ignoring obstacle! Good Luck")
        else:
            pass
    
    def obstacle_near_boundary_where_heading(self):#triggers
        return False

    def no_obstacle_in_direc(self):
        return True
        
    def obstacle_near_ninety_degree(self):
        if(self.vec.mag2d([self.i_current_waypoint[0]-self.px,self.i_current_waypoint[1]-self.py])<15):
            if(self.vec.mag2d([self.i_current_waypoint[0]-self.i_next_waypoint[0],self.i_current_waypoint[1]-self.i_next_waypoint[1]])<8):
                return 1 # means while pitching

            else:
                return 2 #means while rolling
        else:
            return 3        #means not in ninety degree bend

           
        
        

    def basic_guided_switch_and_plan(self):
        if self.relativeAlt>2.5: #todo add current waypoint before to handle edge cases
           
            # bool_temp,self.dist_temp=self.obstacle_in_direction(self.navigation_controller.pos_vector,self.np_mat_to_list,self.desired_ang,self.engaging_distance)
            # self.dist_safe=self.dist_temp-self.safety_margin
           
            temp_wp_dist_vec=[self.i_current_waypoint[0]-self.px,self.i_current_waypoint[1]-self.py] #used to find if the current vector to next waypoint i
             #vector towards next waypoint
            #for handling edge waypoints
            

            if(self.currentMode=='AUTO'):
                #if something is in heading direction, within desired angle and engaging distance, then engage brake/switch to guided
                if(self.vec.mag2d(temp_wp_dist_vec)<40):
                    self.desired_ang=120
                    self.avoidance_mode=2

                else:
                    self.desired_ang=10
                    self.avoidance_mode=1
                bool_temp,self.dist_temp=self.obstacle_in_direction(self.wpvec,self.np_mat_to_list,self.desired_ang,self.engaging_distance) # if any obstacle on the way
                #print('wpvec',self.wpvec,'bool',bool_temp,'distance',self.dist_temp)
                if(bool_temp and self.vec.mag2d(temp_wp_dist_vec)>abs(self.dist_temp)): #if obstacle needs to be avoided
                    self.GuidedMode()
                    print(self.wpvec,'wpvec while seeing obstacles')
                    self.changed_from_auto_to_guided=1
                    
            if(self.currentMode=='GUIDED'and self.changed_from_auto_to_guided==1):
                if(not(self.reaching_A)and not(self.reached_A)):
                    self.plan_and_execute_A()
                if(self.reaching_A  and  not(self.reached_A)):
                    self.check_A()        
                if(self.reached_A):
                    #normal avoidance
                    
                    if(self.avoidance_mode==1):
                        self.normal_avoidance()

    #self.checked is used to ensure that if you have checkked an obstacle is in front then you replan and keep moving

                    elif(self.avoidance_mode==2): #obstacle while moving in longer direction
                        self.edge_case_avoidance()
            
                    else:
                        pass

        # else:
        #     pass

    def check_A(self):
        
        if(abs(self.px-self.guided_target[0])+abs(self.py-self.guided_target[1])<1):
            self.reached_A=True
            self.reaching_A=False
            print('reached A')

    def plan_and_execute_A(self):
        self.dist_safe=self.dist_temp-self.safety_margin #heuristic for safety (5m as of now): sets a point 5 m before obstacle
        self.guided_target= [self.px+self.dist_safe*self.wpvec[0]/self.vec.mag2d(self.wpvec),self.py+self.dist_safe*self.wpvec[1]/self.vec.mag2d(self.wpvec)]
        self.front_stopping=[self.px+(self.dist_temp+3)*self.wpvec[0]/self.vec.mag2d(self.wpvec),self.py+(self.dist_temp+3)*self.wpvec[1]/self.vec.mag2d(self.wpvec)] #added 3 m cause maybe map accuracy meant that itsaw an obstacle nearer, henc evector logic dint work
        
            #keeps sending mavlink command till not switched to guided only if in auto mode
        print('target:A',self.guided_target[0],'---',{self.guided_target[1]})
        self.reaching_A=True
        self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b010111111000), self.guided_target[0], self.guided_target[1], -20, 0, 0, 0, 0, 0, 0, 0, 0))

    def decide_direction(self):
        tempvec=[self.i_next_waypoint[0]-self.i_current_waypoint[0],self.i_next_waypoint[1]-self.i_current_waypoint[1]]
        
        if(np.dot(tempvec,[self.wpvec[1],-self.wpvec[0]])>0):
            self.move_direc=[self.wpvec[1],-self.wpvec[0]]
        else:
            self.move_direc=[-self.wpvec[1],self.wpvec[0]]
            
    
    
    def check_B_edge_case(self):
        if(abs(self.px-self.guided_target[0])+abs(self.py-self.guided_target[1])<2 and self.reaching_B):
            self.checked=1
            self.reached_B=True
            self.reaching_B=False
            print('hey reached B')
    
    
    def edge_case_avoidance(self):   

        
        if(self.direction_decided==0):
            self.decide_direction()
            self.direction_decided=1                      
        else:
            pass
        #todo handle cases where to near to the waypoint causes issues in waypoint change
    
        if(not(self.reached_B) and not (self.reaching_B)and self.direction_decided==1 and self.checked==1):
            self.plan_and_execute_B_edge_case()
        if(not(self.reached_B and self.reaching_B and self.checked==0)):
            self.check_B_edge_case()
        if(self.reached_B):
            self.AutoMode()         
            print('from wp', self.c_point_index)
            self.checker=True
            self.jump_wayPoint(self.c_point_index+2)
            time.sleep(1) #this ensures obtain_current waypoint is updated
            #todo switch to some other logic for current waypoint
            
            
            print('slept :current waypoint', self.i_current_waypoint)
            print('previous waypoint',self.i_previous_waypoint)
            self.heading_vector()
            print('wpvec while switching',self.wpvec)

    def plan_and_execute_B_edge_case(self):
        dist=8 #since swath is 4m
        bool_temp,self.dist_temp=self.obstacle_in_direction(self.move_direc,self.np_mat_to_list,10,dist)
        if(not(bool_temp)): #direction vector and angle
            [x1,y1]=self.i_waypoints[self.c_point_index+1]
            [x2,y2]=self.i_waypoints[self.c_point_index+2]
            px=self.px
            py=self.py
            wp2=self.move_direc[1]
            wp1=self.move_direc[0]
            self.checked=0

            #todo some issue with gazebo direction change on real vehicle
            ftarget= [(px*wp2*x1 - px*wp2*x2 - py*wp1*x1 + py*wp1*x2 + wp1*x1*y2 - wp1*x2*y1)/(wp2*x1 - wp2*x2 - wp1*y1 + wp1*y2),(px*wp2*y1 - px*wp2*y2 - py*wp1*y1 + py*wp1*y2 + wp2*x1*y2 - wp2*x2*y1)/(wp2*x1 - wp2*x2 - wp1*y1 + wp1*y2)]
    
            print('new_target B',self.guided_target[0],'---',{self.guided_target[1]})
            if(self.currentMode=='GUIDED'):
                self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b010111111000), self.guided_target[0], self.guided_target[1], -20, 0, 0, 0, 0, 0, 0, 0, 0))
                self.reaching_B=True
                self.checked=0
        else:
            print('found obstacle in moving')
            pass
            # print('not doing anything or will crash')
    
    def plan_and_move_to_B(self):
        dist=5     
            #checks 90 degree to waypoint vector if an obstacle is there                   
        bool_temp,self.dist_temp=self.obstacle_in_direction([-self.wpvec[1],self.wpvec[0]],self.np_mat_to_list,10,dist)
        if(not(bool_temp)): #direction vector and angle
            self.guided_target= [self.px+dist*(-1)*self.wpvec[1]/self.vec.mag2d(self.wpvec),self.py+dist*self.wpvec[0]/self.vec.mag2d(self.wpvec)]
            print('new_target B',self.guided_target[0],'---',{self.guided_target[1]})
            if(self.currentMode=='GUIDED'):
                self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b010111111000), self.guided_target[0], self.guided_target[1], -20, 0, 0, 0, 0, 0, 0, 0, 0))
                self.reaching_B=True
                self.checked=0
        else:
            print('sees an obstacle towards new target, B and will crash')



    def check_B(self):
        if(abs(self.px-self.guided_target[0])+abs(self.py-self.guided_target[1])<1):
            bool_temp,self.dist_temp=self.obstacle_in_direction([self.wpvec[0],self.wpvec[1]],self.np_mat_to_list,10,10)
            self.checked=1
            if(not(bool_temp)):
                self.reached_B=True
                self.reaching_B=False
                print('hey reached B')
            else:
                self.reached_B=False
                self.reaching_B= False
                print('found obstacle in direction')     

    def plan_and_move_to_C(self):
        bool_temp,self.dist_temp=self.obstacle_in_direction([self.wpvec[0],self.wpvec[1]],self.np_mat_to_list,10,10)
        dist=5
        if(not(bool_temp)and self.dist_temp==100): #direction vector and angle
            self.guided_target= [self.px+dist*self.wpvec[0]/self.vec.mag2d(self.wpvec),self.py+dist*self.wpvec[1]/self.vec.mag2d(self.wpvec)]
            print('new_target C',self.guided_target[0],'---',{self.guided_target[1]})
        
            if(self.currentMode=='GUIDED'):
                self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b010111111000), self.guided_target[0], self.guided_target[1], -20, 0, 0, 0, 0, 0, 0, 0, 0))
                self.reaching_C=True
                self.checked=0
        else:
            print('something in front')
            self.reaching_B=self.reached_B=self.reaching_C=False
            self.checked=1        

    def check_C(self):
        if(abs(self.px-self.guided_target[0])+abs(self.py-self.guided_target[1])<4 and self.reaching_C):
            bool_temp,self.dist_temp=self.obstacle_in_direction([self.wpvec[1],self.wpvec[0]],self.np_mat_to_list,10,10)
            self.checked=1
            if(not(bool_temp)and np.dot(self.wpvec,[self.front_stopping[0]-self.px,self.front_stopping[1]-self.py])<0):
                self.reached_C=True
                self.reaching_C=False
                print('hey reached C')
            else:
                self.reached_C=False
                self.reaching_C= False

    def plan_and_move_to_D(self):
        [x1,y1]=self.i_previous_waypoint
        [x2,y2]=self.i_current_waypoint
        x3=self.px
        y3=self.py              
        self.guided_target[0] = (x3*x1*x1 - 2*x3*x1*x2 - x1*y1*y2 + y3*x1*y1 + x1*y2*y2 - y3*x1*y2 + x3*x2*x2 + x2*y1*y1 - x2*y1*y2 - y3*x2*y1 + y3*x2*y2)/(x1*x1 - 2*x1*x2 + x2*x2 + y1*y1 - 2*y1*y2 + y2*y2)
        self.guided_target[1] = (x1*x1*y2 - x1*x2*y1 - x1*x2*y2 + x3*x1*y1 - x3*x1*y2 + x2*x2*y1 - x3*x2*y1 + x3*x2*y2 + y3*y1*y1 - 2*y3*y1*y2 + y3*y2*y2)/(x1*x1 - 2*x1*x2 + x2*x2 + y1*y1 - 2*y1*y2 + y2*y2)
            # self.guided_target[0]=self.front_stopping[0]+self.wpvec[0]*np.dot(self.wpvec,[self.px-self.front_stopping[0],self.py-self.front_stopping[1]])
            # self.guided_target[1]=self.front_stopping[1]+self.wpvec[1]*np.dot(self.wpvec,[self.px-self.front_stopping[0],self.py-self.front_stopping[1]])
        if(self.currentMode=='GUIDED'):
            self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b010111111000), self.guided_target[0], self.guided_target[1], -20, 0, 0, 0, 0, 0, 0, 0, 0))
            self.reaching_D=True
            self.checked=0
            print('new_target D',self.guided_target[0],'---',{self.guided_target[1]})
    
    def check_D(self):
       
        if(abs(self.px-self.guided_target[0])+abs(self.py-self.guided_target[1])<1):
            print('reached D')
            self.reached_D=True
            self.reaching_D=False
            self.checked=1        

    def normal_avoidance(self):
        if(not(self.reached_B) and not (self.reaching_B) and self.checked==1): #if it has been checked that it has not reached B, but reached the instructed waypoint B
            self.plan_and_move_to_B()
        if(not(self.reached_B) and (self.reaching_B) and self.checked==0):
            self.check_B()[self.wpvec[0],self.wpvec[1]],self.np_mat_to_list,10,10
        if(self.reached_B and not(self.reaching_C) and not(self.reached_C)and self.checked==1):
            self.plan_and_move_to_C()
        if(not(self.reached_C) and (self.reaching_C) and self.checked==0):
            self.check_C()
        if(self.reached_C and not self.reaching_D and self.checked==1):
            self.plan_and_move_to_D()
        if(not(self.reached_D) and (self.reaching_D) and self.checked==0):
            
            self.check_D()

        if(self.reached_D):
            self.AutoMode()         
            print(self.currentMode)



    def  jump_wayPoint(self,waypoint_number):

        #self.add_new_message_to_sending_queue(mavutil.mavlink.mission_set_current_send(self.target_system, self.target_component, waypoint_number))
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_mission_set_current_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                                                  self.mavlinkInterface.mavConnection.target_component,
                                                                                                                  waypoint_number)
                                                              )
        # self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_command_long_message(self.mavlinkInterface.mavConnection.target_system,
        #                                                                                        self.mavlinkInterface.mavConnection.target_component,
        #                                                                                        mavutil.mavlink.MAV_CMD_DO_JUMP, # command
        #                                                                                        0,                                       # confirmation
        #                                                                                        waypoint_number,                                       # param1 
        #                                                                                        repeat_times,                           # param2 
        #                                                                                        0,                                       # param3
        #                                                                                        0,                                       # param4
        #                                                                                        0,                                       # param5
        #                                                                                        0,                                       # param6
        #                                                                                        0)                                      # param7
        # self.mavlinkInterface.mavConnection.mav.command_long_send(self.mavlinkInterface.mavConnection.target_system,
        #                                                           self.mavlinkInterface.mavConnection.target_component,
        #                                                           mavutil.mavlink.MAV_CMD_DO_JUMP,
        #                                                           0,waypoint_number,repeat_times,0,0,0,0,0

#                                                 )
        
        print('jumped to waypoint',waypoint_number)
        
    def guided_point_decide(self,vec_direct):#triggers avoidance
        """
        Basic Stopping Class
        """

        if self.currentMode == 'AUTO' and self.relativeAlt>1:
            if(self.obstacle_in_direction(self.navigation_controller.pos_vector,10)): #direction vector and angle
                #todo add waypoint vector above
            
                self.guide = 1
                print('hey detected obstacle_braking')
                                
            else:
                print("Ignoring obstacle! Good Luck")
        else:
            pass





    

    def Obtain_Waypoints(self):
        #/*/*addd comments
        self.mavlinkInterface.mavConnection.mav.mission_ack_send(self.mavlinkInterface.mavConnection.target_system,self.mavlinkInterface.mavConnection.target_component, 0)
        time.sleep(1)
        print("Trying to send a request ")
        while True:
            try:
                self.mavlinkInterface.mavConnection.waypoint_request_list_send()
                print("Trying to obtain the count ")
                msg = self.mavlinkInterface.mavConnection.recv_match(type=['MISSION_COUNT'],blocking=True,timeout=5)
                self.waypoint_count = msg.count
                break
            except:
                print("Retrying to obtain count")

        time.sleep(1)
        print ("Count is ",self.waypoint_count)
        time.sleep(1)
        c=1
        i = 0
        while(i<self.waypoint_count):
            try:
                c=0
                self.mavlinkInterface.mavConnection.waypoint_request_send(i)
                self.mavlinkInterface.mavConnection.waypoint_request_list_send()
                msg = self.mavlinkInterface.mavConnection.recv_match(type=['MISSION_ITEM'],blocking=True,timeout=1.5)
                time.sleep(0.15)
                print ('Receving waypoint {0}'.format(msg.seq))    
                self.waypoints.append((msg.x,msg.y))
                i=i+1
            except:
                print("Missed Waypoint, retrying waypoint",i)
        self.mavlinkInterface.mavConnection.mav.mission_ack_send(self.mavlinkInterface.mavConnection.target_system,self.mavlinkInterface.mavConnection.target_component, 0) # OKAY
        print(self.waypoints)#Contains all the waypoints of the mission in lat and long format!!
        self.homeLocation = self.waypoints[0]
        #/*/*get mission file from vinay
        #self.i_waypoints.append((0,0))
        #todo check local waypoint coordinates
        for i in range(0,self.waypoint_count,1):
            local_coord=pm.geodetic2enu(self.homeLocation[0],self.homeLocation[1],100,self.waypoints[i][0],self.waypoints[i][1],100)
            self.i_waypoints.append((-local_coord[1],-local_coord[0])) #/*/*change for real drone
        print(self.i_waypoints)


    # def handle_mission_current(self,msg,nextwaypoint):
    #     #/*/*check
    #     if msg.seq > nextwaypoint:
    #         self.c_point_index = int(msg.seq)
    #         print(self.c_point_index)
    #         #print ("Moving to waypoint ",self.c_point_index,self.waypoints[self.c_point_index])
    #         self.current_waypoint = self.waypoints[self.c_point_index]
    #         self.i_current_waypoint = self.i_waypoints[self.c_point_index]
            
                        
    #         nextwaypoint = msg.seq+1
    #         if(nextwaypoint<len(self.waypoints)):
    #             self.next_waypoint = self.waypoints[nextwaypoint]
    #             self.i_next_waypoint = self.i_waypoints[nextwaypoint]
    #             print('current waypoint',self.i_current_waypoint)
    #         #    print ("Next Waypoint ", self.next_waypoint)
    #             return nextwaypoint
    #         else:
    #          #   print("Last Waypoint Reaching")
    #             return -1


    def handle_mission_current(self,msg,nextwaypoint):
        try:
            if msg.seq > nextwaypoint:
                self.c_point_index = int(msg.seq)
                self.p_point_index = int(msg.seq)-1
                if self.p_point_index < 0: #Handle 1st waypoint
                    self.p_point_index = 0
                #print(self.self.c_point_index,self.p_point_index)
                #print ("Moving to waypoint ",self.c_point_index,self.waypoints[self.c_point_index])
                self.current_waypoint = self.waypoints[self.c_point_index]
                self.previous_waypoint = self.waypoints[self.p_point_index]
                self.i_current_waypoint = self.i_waypoints[self.c_point_index]
                self.i_previous_waypoint = self.i_waypoints[self.p_point_index]
                if(self.checker):
                    print('inside obtain current waypoint: current waypoint,previous waypoint,current waypoint number',self.i_current_waypoint,'--',self.i_previous_waypoint,'---',self.c_point_index)

                nextwaypoint = msg.seq+1
                if(nextwaypoint<len(self.waypoints)):
                    self.next_waypoint = self.waypoints[nextwaypoint]
                    self.i_next_waypoint = self.i_waypoints[nextwaypoint]
                    #print ("Next Waypoint ", self.next_waypoint)
                    return nextwaypoint
                else:
                    #print("Last Waypoint Reaching")
                    return -1
        except:
            pass
            print("Error with code in handle mission current")


    def Obtain_Current_Waypoint(self):
        nextwaypoint=0
        while True:
            #/*/*check
            msg = self.mavlinkInterface.mavConnection.recv_match(type=['GLOBAL_POSITION_INT', 'MISSION_CURRENT', 'MISSION_COUNT', 'HEARTBEAT'],blocking=True,timeout=1.5)
            self.mavlinkInterface.mavConnection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 192, 0, 4)
            if(msg!=None and msg.get_type()!=None):
                try:
                    if (nextwaypoint==0 and self.c_point_index<2):
                        #todo see why the issue is
                        
                        self.current_waypoint=self.waypoints[1]
                        self.next_waypoint=self.waypoints[2]
                        self.i_current_waypoint = self.i_waypoints[1]
                        self.i_next_waypoint = self.i_waypoints[2]
                    if msg.get_type() == 'HEARTBEAT':
                        self.mavlinkInterface.mavConnection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 192, 0, 4)
                    if msg.get_type() == 'GLOBAL_POSITION_INT':
                        relative_alt = msg.relative_alt
                    if msg.get_type() == 'MISSION_CURRENT' and nextwaypoint!=None: 
                        nextwaypoint = self.handle_mission_current(msg,nextwaypoint)
                        #print ('Next Waypoint', nextwaypoint,self.next_waypoint)
                        #print ("Current Waypoint",self.current_waypoint)
                        # if nextwaypoint == None:
                        #     nextwaypoint = -1 #Need to handle None type
                        if nextwaypoint != None:
                            if nextwaypoint >= self.waypoint_count - 1:
                                if relative_alt<=1*1000*0.05: 
                                    print ("Reached land altitude")
                                    break
                    elif nextwaypoint == None:
                        #print("Generated next waypoint was none, so skipped updating the current waypoint and next waypoint.")
                        pass
                    time.sleep(0.1)
                    #print('next wp',self.nextwaypoint)
                    #/*/*check
                    #self.mavlinkInterface.mavConnection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 192, 0, 4) # send heart beat to airframe per 1 sec

                except KeyboardInterrupt:
                    print("Exception in obtain_Current_Waypoint")

    
    
    
    def DijKstras_Triggerer(self,world_map): #This needs to be called everytime. Need to set a trigger such that I can call it after it has completed its execution Part
        #Fetch the local Obstacle list and read sensor Data Here:
        #self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_fupdate_miss_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), 2, -41, -10, 0, 0, 0, 0, 0, 0, 0, 0))
        
        if(len(world_map)!=0):

            
            ## Obstacle detection
            # Detection for Brake Command
            self.navigation_controller.basic_stop()
            # Brake command
            self.handbrake()
            
            
            length = 40
            breadth = 40
            local_obstacle_list = []
            for c in world_map: #Obtain the List of proximity Obstacles in The Grid
                
                if(abs(c[0]-self.px)<breadth/2 and abs(c[1]-self.py)<length/2):
                    
                    local_obstacle_list.append(c)

            if len(local_obstacle_list)==0:
                print("No Local Obstacles from the map here")

            else:
                for c in local_obstacle_list:# Checks if any obstacle in that grid is in the viscinity of 2x2 from the drone
                    #/*/*missionplanner parameters and delay mhr
                    if((abs(c[0]-self.px)<=7 and abs(c[1]-self.py)<=7) ):
                        if(self.currentMode == 'AUTO'):
                            self.brakeMode()#Apply hand break to start guided navigation
                            #/*/* check flags
                            
                            print("Obstacle seen in local map,Gunna Switch to Guided")
                            time.sleep(2)
                            if self.currentMode == 'BRAKE':
                                print("Braked!!")
                            self.GuidedMode()
                            if self.currentMode == 'GUIDED':
                                print("Guided Mode!!")
                                time.sleep(5)
                                #/*/* add parameters for distance and tolerance
                            gx,gy = self.DijKtras_Map_Goal_Generation(local_obstacle_list,length,breadth)                
                            self.Dijktras_Manueaver_Path_Generation(local_obstacle_list,int(gx),int(gy))
                            self.dijktras_maneaveur()
                            print('Completed Dijtraks Maneavuer, switching to Auto')
                            self.AutoMode()
                        #else:
                            #print("The Drone was not in Auto mode to start Obstacle Avoidance")
                        break

    def Testing_Waypoint(self): #This is just for me to obtain testing waypoints
        c = self.i_waypoints
        for i in c:
            while((self.px-i[0])*(self.px-i[0])+(self.py-i[1])*(self.py-i[1])>=1):
                if(self.currentMode=='GUIDED'):
                    self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), i[0], i[1], -10, 0, 0, 0, 0, 0, 0, 0, 0))
                else:
                    override = 1
                    print("Not in GUIDED, Pilot Probably Overrided")
                    break
        pass

    def brakeMode(self):
        if self.initvar==1:
            print("Handbrake Initialize")
            self.initvar = 0
        #Only brake when previously brake command is not given, altitude is greater than 1
        #/*/*check logic 
        #while not self.brake and self.alreadybraked and not self.relativeAlt>1:
        while self.currentMode!='BRAKE':
            print("Switching to Brake Mode")
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                                           17))
        #self.alreadybraked = 1

    def GuidedMode(self):
        while (self.currentMode != 'GUIDED' and self.currentMode=='AUTO'):
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system, 
                                                                                                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                                                        4))
            time.sleep(0.05)
        print("switched To Guided")

    def AutoMode(self):
        while self.currentMode != 'AUTO':
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system, 
                                                                                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                                           3))
            time.sleep(0.05)
        print("switched To Auto")
#whenever in auto reset all the markers so that planning can be started again.
        self.reached_A=self.reached_B=self.reached_C=self.reached_D=self.reaching_C=self.reaching_B=self.reaching_D=self.reaching_A=False
        self.changed_from_auto_to_guided=0
        self.checked=1
        self.avoidance_mode=0
        print('flags reset, again auto')



    def check_sensor_Raw(self):  #Triggers avoidance if the front portion of the sensor detects obstacle under 5m
        while True:
            try:
                #/*/*add heading direction
                
                for i in self.lidar.raw_data[20:45]:
                    if i<15:
                        self.sensor_trig=1
                        break
                    else:
                        self.sensor_trig=0
            except:
                pass

    def DijKtras_Map_Goal_Generation(self,local_obstacle_list,l,b):#Finds the goal coordinate where the drone must go to in that grid using Vectors
        print("Current inertial Waypoint",self.i_current_waypoint)
        Way_Point_X = self.i_current_waypoint[0]
        Way_Point_Y = self.i_current_waypoint[1]
        Vx = Way_Point_X - self.px
        Vy = Way_Point_Y - self.py
        Mag = math.sqrt(Vx*Vx + Vy *Vy)
        Vx = Vx/Mag #Unit Vector Along X and Y
        Vy = Vy/Mag
        goal_x,goal_y = 0,0#Store initial goal here
        print("Entered into Dijktras Goal Generation ")
        for i in range(10,40,1): #Tries to Place the goal coordinate about 8m away in the heading direction.
            flag = 1
            Vx = int(i*Vx) + int(self.px)
            Vy = int(i*Vy) + int(self.py)
            print(Vx,Vy)
            dis = []
            for j in local_obstacle_list:
                dis.append((j[0]-Vx)*(j[0]-Vx)+(j[1]-Vy)*(j[1]-Vy))
            for j in dis:
                if(j<10):
                    flag = 0
            if(flag == 1):
                print("GOAL Possible!!")
                print("Goal at ",Vx,Vy)
                if((self.px-int(b/2)<Vx<self.px+int(b/2)) and (self.py-int(b/2)<Vy<self.py+int(b/2))):#Checks if the Goal Coordinate is within the boundary
                    goal_x,goal_y = Vx,Vy
                    break
                else:
                    print("Goal Generated Not in Viscinity, Use Next WayPoint.....To be Done here")
                    Vx = Vx/i
                    Vy = Vy/i
                    goal_x,goal_y = Vx,Vy
                    
            else:
                Vx = Way_Point_X - self.px
                Vy = Way_Point_Y - self.py
                Vx = Vx/Mag
                Vy = Vy/Mag
                print("Generated Goal near obstacle, re-generating")
        return(int(goal_x),int(goal_y))
            
    def Dijktras_Manueaver_Path_Generation(self,obs,goal_x,goal_y):#Pass obstacles x and  y near that proximity
        ox = []
        oy = []
        time_start = time.time()
        for i in obs:
            ox.append(i[0])
            oy.append(i[1])
        
        gx = goal_x
        gy = goal_y
        length = 60
        width = 60
        l_behind = int(length/2)
        l_forward = int(length/2)
        w = int(width/2)
        sx= int(self.px)
        sy = int(self.py)

        for i in range(sx-w, sx+w):#Width Boundaries
            ox.append(i)
            oy.append(sy-l_behind)
        for i in range(sx-w,sx+w):
            ox.append(i)
            oy.append(sy+l_forward)

        for i in range(sy-l_behind, sy+l_forward):#Length Boundaries
            ox.append(sx-w)
            oy.append(i)
        for i in range(sy-l_behind, sy+l_forward):
            ox.append(sx+w)
            oy.append(i)


        print("Starting Dijkstra's Manueaver")
        grid_size=2
        robot_radius=2
        self.path_planned = self.dijktras_PathPlanning(ox, oy, grid_size, robot_radius,sx, sy, gx, gy)
        self.path_planned = self.path_planned[::-1]
        print("Time taken to generate the local path ",time.time()-time_start)


    def dijktras_maneaveur(self):
        override = 0
        c = self.path_planned
        for i in c:
            while((self.px-i[0])*(self.px-i[0])+(self.py-i[1])*(self.py-i[1])>=1):
                if(self.currentMode=='GUIDED'):
                    self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), i[0], i[1], -10, 0, 0, 0, 0, 0, 0, 0, 0))
                else:
                    override = 1
                    print("Not in GUIDED, Pilot Probably Overrided")
                    break
            if override == 1:
                break
        if override==0:
            self.AutoMode()#Switch to AutoMode after Dijktras manueaver
        else:
            pass

    def handbrake(self):
        
        if self.brake and not self.alreadybraked and self.relativeAlt>1 and self.currentMode=='AUTO':
            print("Switching to Brake Mode")
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                                           17))
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                                           17))
            
            self.alreadybraked = 1
           
    def trigger_avoidance(self):
        '''Triggers Guided Mode only when drone is already in Brake mode due to Basic_stop() and Guiding is not started'''
        ######################## Equating SAAController's heading with updated heading by SAAHandling #####################
        try:
            heading = np.asarray(self.coordinate_transform.heading)
            #print(heading)
            self.navigation_controller.heading = np.array([heading[0][0], heading[0][1]])
        except:
            pass
        ###################################################################################################################

        ################################# Change to GUIDED mode only if it first braked  #############################################
        if self.currentMode == 'BRAKE':
            while not self.navigation_controller.guide:
                self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system, 
                                                                                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                                            4))
                self.t3 = time.time()
            print('Switched to Guided Mode for Avoidance')
            self.check_mode(True)
            self.navigation_controller.guiding = 1       # This logic is seperately included because guiding should be true only when in guided mode after it was braked
        ########################################################################################################################
                    
        ############################ Method to switch back to AUTO mode after avoidance is done #####################################
        if self.navigation_controller.auto:
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system, 
                                                                                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                                           3))
            print('Forcing AUTO mode')
            if self.currentMode == 'AUTO':  # Why there was != and still it worked
                self.navigation_controller.auto = 0
                # print(f'time(t1) ---> {self.navigation_controller.t1}  t2 --> {self.t2}  t3 --> {self.t3}  t4 --> {self.t4}   t5 --> {self.navigation_controller.t5}   t6 --> {self.t6}')
                # print('----------------------')
                self.g = 0
                self.v = 0
                self.check_mode(True)
                ##print('Entered AUTO mode')
        ###########################################################################################################################

    def check_mode(self, out):
        
        mode = self.currentMode
        if out:
            print('Mode : ', mode)

        if mode == 'GUIDED':
            self.navigation_controller.guide = 1
              
        else:
            self.navigation_controller.guide = 0
            self.navigation_controller.avoided = 0
            self.navigation_controller.avoiding = 0

        if mode == 'AUTO':
            self.navigation_controller.ctrl = 0
            self.alreadybraked = 0     # To revert back the default value so that again the process if necessary can be repeated
            self.overriding = 0
            # Getting pathplanning() ready for next obstacle
            self.t = 0

        ###### If pilot tries to switch to any mode except GUIDED and AUTO, stop everything in code ######
        # if mode == 'LOITER':
        #     self.terminate = 1
        if mode !='AUTO' and mode !='GUIDED' and mode !='BRAKE':
        #if mode =='LOITER':
            self.overriding = 1
            ##print('Pilot Overrided')
            # If pilot has overrided, then change all triggers to default values
            if self.overriding:
                self.navigation_controller.brake = 0
                self.navigation_controller.guide = 0              
                self.navigation_controller.guiding = 0            
                self.navigation_controller.auto = 0               
                self.navigation_controller.ctrl = 0               
                self.navigation_controller.stop = 0
                self.navigation_controller.avoided = 0            
                self.navigation_controller.avoiding = 0
            

    def maneuver(self): 
        if self.navigation_controller.ctrl and self.navigation_controller.guide:
            # Navigate rightward unless drone sees safe angle
            if not self.navigation_controller.stop and not self.navigation_controller.avoided:
                # self.g = self.g + 1
                # if self.g == 1:
                #     self.t4 = time.time()
                self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b010111000111), 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0))
                print('Moving right !!!')

            # Stop command in Guided mode
            if self.navigation_controller.stop and not self.navigation_controller.avoided:
                # self.v = self.v + 1
                # if self.v == 1:
                #     self.t6 = time.time()
                self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b010111000111), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                #print('Forced Stop !')

            # Moving Forward command in Guided mode
            #print(f'                                                  {not self.navigation_controller.stop and self.navigation_controller.avoided}')
            if not self.navigation_controller.stop and self.navigation_controller.avoided:
                #print('                                                                                      Forward movement maneuver condition')
                self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111000111), 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0))
                print('Moving forward !!!')

            # Moving left command in Guided mode
            if self.navigation_controller.return_track and self.navigation_controller.avoided and self.navigation_controller.stop:
                #X is North and Y is neg East but self.py and y coord of obstacle_map is already in neg East
                N = self.navigation_controller.target[0] - self.px
                E = self.navigation_controller.target[1] - self.py
                vel_r = 4*[N, E]/np.linalg.norm([N, E])
                #self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b110111111000), N, E, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b010111000111), 0, 0, 0, vel_r[0], vel_r[1], 0, 0, 0, 0, 0, 0))

    def planned_maneuver(self): 
        if self.plan.plan_command:
            # Move towards A point
            if not self.A_reached:
                A_drone_cent = self.navigation_map.convert_inertial_to_rel_avoid(self.plan.A_pos_origin_cent)
                A_drone_cent = 2.5*A_drone_cent/np.linalg.norm(A_drone_cent)
                self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b010111000111), 0, 0, 0, A_drone_cent[0], A_drone_cent[1], 0, 0, 0, 0, 0, 0))
                if abs(self.px - self.plan.A_pos_origin_cent[0]) <= 2 and abs(self.py - self.plan.A_pos_origin_cent[1]) <= 2:
                    # Will act as a Brake
                    self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b010111000111), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                    self.A_reached = 1
                    print('Reached point A')

            if self.A_reached and not self.B_reached:
                B_drone_cent = self.navigation_map.convert_inertial_to_rel_avoid(self.plan.B_pos_origin_cent)
                B_drone_cent = 2.5*B_drone_cent/np.linalg.norm(B_drone_cent)
                self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b010111000111), 0, 0, 0, B_drone_cent[0], B_drone_cent[1], 0, 0, 0, 0, 0, 0))
                if abs(self.px - self.plan.B_pos_origin_cent[0]) <= 2 and abs(self.py - self.plan.B_pos_origin_cent[1]) <= 2:
                    # Will act as a Brake
                    self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b010111000111), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                    self.B_reached = 1
                    print('Reached point B')

                
            if self.B_reached and not self.C_reached:
                C_drone_cent = self.navigation_map.convert_inertial_to_rel_avoid(self.plan.C_pos_origin_cent)
                C_drone_cent = 2.5*C_drone_cent/np.linalg.norm(C_drone_cent)
                self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b010111000111), 0, 0, 0, C_drone_cent[0], C_drone_cent[1], 0, 0, 0, 0, 0, 0))
                if abs(self.px - self.plan.C_pos_origin_cent[0]) <= 2 and abs(self.py - self.plan.C_pos_origin_cent[1]) <= 2:
                    # Will act as a Brake
                    self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b010111000111), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                    self.C_reached = 1
                    print('Reached point C')

            if self.C_reached:
                self.plan.plan_command = 0
                self.A_reached = 0
                self.B_reached = 0
                self.C_reached = 0
                self.navigation_controller.brake = 0
                self.navigation_controller.auto = 1
                self.navigation_controller.guiding = 0

            # self.brake = 0
            # self.navigation_controller.guide = 0              
            # self.navigation_controller.guiding = 0            
            # self.navigation_controller.auto = 1               
            # self.navigation_controller.ctrl = 0               
            # self.navigation_controller.stop = 0
            # self.navigation_controller.avoided = 0            
            # self.navigation_controller.avoiding = 0
    
    def termination(self):
        '''Flight Test safety purpose'''
        if self.terminate == 1: 
            for task in self.scheduledTaskList:
                task.stop()
            print('Terminated')  
        

    def previous_position_storer(self):
        """Warning: This function should be called at a co-prime pair frequency with update_vars
        The function relies on the fact that the current value has already been updated and in the update_vars and the 
        previous position is stored for 1 second. 
        [Tested in SITL]
        """
        self.prev_px = self.px
        self.prev_py = self.py

    def debug(self):
        """Debugger if you want to print something
        """
        #print(f"New:  {self.navigation_controller.obstacle_map}")
        # for i in range(self.navigation_controller.obstacle_map):
        #     obstacle = [self.navigation_controller.obstacle_map[i,0], self.navigation_controller.obstacle_map[i,1]]
        #plt.clf()
        print(self.scheduledTaskList)

        


    def update_vars(self):
        """This function acts as a bridge between different class to transfer data. Part of the requirements for 
        developing the algorithm
        """
        #6.198883056640625e-06 seconds
        self.front_sensor.data = self.lidar.raw_data
        #print(self.front_sensor.data)

        #Gazebo axes are different from my 
        # axis, thus I have to handle it. 
        if self.lidar.drivername == 'SITL':
            self.coordinate_transform.roll = self.roll + math.pi
        else:
            self.coordinate_transform.roll = self.roll
        self.coordinate_transform.pitch = self.pitch
        #Yaw wrapper to keep it compatible with my transformations
        self.coordinate_transform.yaw = math.atan2(math.sin(self.yaw),math.cos(self.yaw))
        self.coordinate_transform.px = self.px
        self.coordinate_transform.py = self.py
        self.coordinate_transform.pz = self.relativeAlt
        self.navigation_controller.px = self.px
        self.navigation_controller.py = self.py
        self.navigation_controller.vx = self.vx
        self.navigation_controller.vy = self.vy
        self.brake = self.navigation_controller.brake
        self.coordinate_transform.x = self.front_sensor.X
        self.coordinate_transform.y = self.front_sensor.Y
        self.navigation_controller.mode = self.currentMode
        self.navigation_map.px = self.px
        self.navigation_map.py = self.py

        self.navigation_controller.prev_px = self.prev_px
        self.navigation_controller.prev_py = self.prev_py
        # Pilot overriding logic
        self.navigation_controller.overriding = self.overriding

        self.plan.px = self.px
        self.plan.py = self.py
        if self.plan.home_pos.size == 0:
            self.plan.home_pos = self.home
        self.plan.w_vector = self.wp

    def dijktras_PathPlanning(self,ox, oy, grid_size, robot_radius,sx, sy, gx, gy):#Obstacle list X coords, Obstacle list Y coords, GridSize,RobotRadius,StartX, StartY,GoalX,GoalY
        dijkstra = Dijkstra(ox, oy, grid_size, robot_radius)
        rx, ry = dijkstra.planning(sx, sy, gx, gy)
        return(list(zip(rx,ry)))



    def update_mission(self):                                             
        self.front_sensor.handle_raw_data() #returns x and y coordinates of only masked points
        self.update_vars() #for transferign variable values across classes
        self.coordinate_transform.x = self.front_sensor.X
        self.coordinate_transform.y = self.front_sensor.Y  
        #print(self.front_sensor.X)
       
        self.coordinate_transform.update_vehicle_states()#gets heading and rotation matrix
       
    
        #self.heading_vector() #gives heading of drone based on waypoints some other logic can also be interpreted
        stop_flag=0
        min_dist=40
        temp_wp_dist_vec=[self.i_current_waypoint[0]-self.px,self.i_current_waypoint[1]-self.py] #used to find if the current vector to next waypoint i
        #todo add extra logic to handle other side
        if(np.size(self.front_sensor.X)!=0 and self.currentMode=='AUTO'):
            print('seeing something here')
            # print(self.front_sensor.X,self.front_sensor.Y)
            #angle_zero_lidar=self.yaw-math.atan(self.wpvec[1]/self.wpvec[0])

            for i in range(0,np.size(self.front_sensor.X)):
                
                #temp_ang=abs(180*math.atan2(self.front_sensor.Y[i],self.front_sensor.X[i])/3.14) #gives -pi/2 to +pi/2
                #for angular avoidance while not just pitching
               
                #logic for moving at some angle

                
                
                if(abs(temp_ang)<self.obstacle_stopping_angle):
                    if(self.vec.mag2d(temp_wp_dist_vec)>self.vec.mag2d([self.front_sensor.X[i],self.front_sensor.Y[i]]) and self.currentMode=='AUTO'):
                        stop_flag=1
                        
                    else:
                        print('obstacle in stopping angle but after waypoint')
                else:
                    print('ignored')
            
            if(stop_flag==1):
                while(self.currentMode!='BRAKE' and self.currentMode=='AUTO'):
                    self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,17))
                    print('message for brake sent')                                                                            
                    time.sleep(0.1)
        else:
            pass
            
            

               


        #
        # self.coordinate_transform.rotate_body_to_inertial_frame() #returns 3xn matrix containing n obstacles, in inertial frame, on drone origin
        # # Create Obstacle map out of obstacle vector
        # self.navigation_controller.obstacle_map=None #fetch a new map everytime
        # #print(self.coordinate_transform.obstacle_vector_inertial)
        # if self.coordinate_transform.obstacle_vector_inertial.size != 0: #inertial map updation logic
            
            
            
            
            
            
            
        #     if(self.relativeAlt>1):
        #         self.navigation_stack() #self.matmap is world map matrix, in inertial coordinates
        # self.count_dum=self.count_dum+1 
        
        # self.local_fetcher() #fetches local 2d occupancy grid        
        # self.local_mat_to_list() #converts into list[[1,2]] Gives inertial coordinates of nearby obstacles
        # self.np_mat_to_list=np.array(self.mat_list) #get obstacles in world frame as np array uncomment this for real time avoidance
            
        
        
        
        # #self.np_mat_to_list=np.array([[0,-75],)
        
        # #self.np_mat_to_list=np.array([[-0,-30]])
        # #assume that list is correct
       
        # #todo instead of velocity use waypoint vector?

        # #self.np_mat_to_list=np.array([[0,-149],[-5,-150],[5,-150],[0,-221],[0,-226],[-5,-221],[5,-226],[0,-297],[0,-304],[5,-297],[-5,-304]])
        # #switching logic for auto

        # if(np.size(self.np_mat_to_list)!=0):
        #         #todo handle minimum distance from obstacle
        #     self.basic_guided_switch_and_plan() #Only works in auto
                
        #         # Brake command
        
    def heading_vector(self):
        #todo separate logic for rtl
        vx2=self.vx*self.vx
        vy2=self.vy*self.vy
        if(vx2+vy2>1):
            self.wpvec=np.array([self.vx,self.vy])/(math.sqrt(vx2+vy2))
            

            # print('waypoints not fetched, erorr in finding heading vector')
            # print('current waypoint',self.i_current_waypoint)
            # print('previous waypoint',self.i_previous_waypoint)

            #todo add some logic here



                     
    def obstacle_storing_map(self):# Storing previously detected obstacles inertial position in self.obstacle_inertial, do it only when guiding = True
        if self.obstacle_map_optim.size != 0:
            # This if condition was there for Obstacle_detection() method in SAAController
            if self.navigation_controller.obstacle_inertial.size == 0:
                self.navigation_controller.obstacle_inertial = np.array([[self.coordinate_transform.px,self.coordinate_transform.py]])
                #self.navigation_controller.obstacle_inertial = np.array([[0,0]])
            
            # Below code takes greatest integer smaller than individual obs_vec pos for the sake of removing multiple points for single obstacle problem
            if True:
                self.navigation_controller.obstacle_inertial = np.unique(np.concatenate((self.navigation_controller.obstacle_inertial,self.obstacle_map_optim),axis=0),axis=0)
                #Below code is required because while changing from relative to inertial origin center, noises again enter thru drone pos and so same obstacle would be stored with various points
                t = 0
                sum = 0
                obs_mean = []
                for i in range(len(self.navigation_controller.obstacle_inertial)-1):
                    vec = self.navigation_controller.obstacle_inertial[i]
                    if np.any(abs(vec-self.navigation_controller.obstacle_inertial[i+1])<[3,3]):
                        t = t+1
                        sum = sum + vec
                    elif t != 0:
                        mean_vec = sum/t
                        t = 1
                        sum = vec
                        obs_mean.append(mean_vec)
                    else:
                        obs_mean.append(vec)
                if len(obs_mean) == 0 and t != 0:
                    self.navigation_controller.obstacle_inertial = (sum/t).reshape(1,2)
                else:
                    self.navigation_controller.obstacle_inertial = np.unique(np.array(obs_mean), axis=0)
                #self.navigation_controller.mag_obs_inertial = np.array([self.vec.mag2d(self.navigation_map.convert_inertial_to_rel_avoid(i)) for i in self.navigation_controller.obstacle_inertial])

                self.navigation_controller.obstacle_body = np.array([self.navigation_map.convert_inertial_to_rel_avoid(i) for i in self.navigation_controller.obstacle_inertial])
        else:
            pass
           
    def map_optimize(self):
        t = 0
        sum = 0
        obs_mean = []
        for i in range(len(self.navigation_controller.obstacle_map)-1):
            # vec and vec_next are origin centered
            vec = self.navigation_map.convert_rel_to_inertial_avoid(self.navigation_controller.obstacle_map[i])
            vec_next = self.navigation_map.convert_rel_to_inertial_avoid(self.navigation_controller.obstacle_map[i+1])
            if np.any(abs(vec-vec_next)<[2,2]):
                t = t+1
                sum = sum + vec
            elif t != 0:
                mean_vec = sum/t
                t = 1
                sum = vec
                obs_mean.append(mean_vec)
            else:
                obs_mean.append(vec)
        if len(obs_mean) == 0 and t != 0:
            self.obstacle_map_optim = np.array(sum/t).reshape(1,2) 
        else:   
            self.obstacle_map_optim = np.unique(np.array(obs_mean), axis=0)

    def navigation_stack(self):
        #Angular inertial obstacle vector to grid based reading and that is stored in global map
        #self.navigation_map.convert_rel_obstacle_to_inertial(self.navigation_map.grid(self.coordinate_transform.obstacle_vector_inertial.T))
         #updates map in world frame   
             
        self.navigation_map.map=self.navigation_map.convert_rel_obstacle_to_inertial1(self.coordinate_transform.obstacle_vector_inertial.T[:,0:2])#inertial frame coordinates in origin
        #self.navigation_map.inertial_grid=np.unique(np.concatenate((self.navigation_map.map, self.navigation_map.inertial_grid),axis=0),axis=0)
        #Relative obstacle is sent to the navigation algorithm
        #**todo send this map to update world map, and extract obstacle map from it
        #add a variable for world_map      
        #print(self.navigation_map.map)
        #adds dummy navigation_map
        
        for dummat in self.navigation_map.map:
            #updates a grid of size field_x and field_y with drone's starting position at the centre of the field.
            self.matmap[int(np.round(self.field_y/2-dummat[0,1]))][int(np.round(dummat[0,0]+self.field_x/2))]=1 
       
        




    def local_fetcher(self):
        #this function fetches map of size length_fetcher and width_fetcher from the gridf around position of the drone
        self.relmatmap=self.matmap[int(np.ceil(self.field_y/2-self.py-self.length_fetcher/2)):int(np.ceil(self.field_y/2-self.py+self.length_fetcher/2)),int(np.ceil(self.px+self.field_x/2-self.width_fetcher/2)):int(np.ceil(self.px+self.field_x/2+self.width_fetcher/2))]
        self.res=1
    def local_mat_to_list(self):
        #this function converts fetched grid obstacles into x and y coordinates arrays stored as a list in mat_list
        self.mat_list=[]
        for i in range(0,len(self.relmatmap)):#no of rows
            for j in range(0,len(self.relmatmap[0])):
                if (self.relmatmap[i][j]==1):
                    self.mat_list.append([-self.width_fetcher/2+j*self.res+self.px,self.length_fetcher/2-i*self.res+self.py])# y is absolute,x is

    def set_data_stream(self):
        # data rate of more than 100 Hz is not possible as recieving loop is set to run at interval of 0.01 sec
        # don't change that as it affects other vehicles as well
    
        # request data to be sent at the given rate
        # arguments are (target_system, target_component, stream_type, frequency in Hz, stop(0) or start(1))
    
        # stop data which are coming automatically to stop recieving unnecessary messeges
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 0))


        #self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1))
        #self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 1, 1))
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1))
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 2, 1))
        #self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 1, 1))
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5, 1))
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 1, 1))
        #self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 1, 1))
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 2, 1))
    
        logging.info("Stream Rate have been set")
    
    def handle_recieved_message(self):
        while True:
            if self.killAllThread.is_set():
                break
            recievedMsg = self.get_new_message_from_recieving_queue()
            if recievedMsg is not None:
                super().handle_recieved_message(recievedMsg)
            else:
                time.sleep(0.01)
            
    def kill_all_threads(self):
        logging.info("TestCompanionComputer killing all threads")
        super().kill_all_threads()
#        self.killAllThread.set()
#        
#        for task in self.scheduledTaskList:
#            task.stop()
#        
        self.handleRecievedMsgThread.join()
        logging.info("TestCompanionComputer joined all threads")