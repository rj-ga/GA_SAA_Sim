#!/home/dhruv/.local/lib/python3.8
# -*- coding: utf-8 -*-
"""
Created on Wed Jul  8 14:07:56 2020

@author: Sachchit Vekaria
@Organization: General Aeronautics Pvt Ltd
"""

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

import numpy as np
import sys
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

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



        # Terminate trigger
        self.terminate = 0    
        
        # Pilot overriding
        self.overriding = 1    # Initially pilot control
        self.dist_temp=0

        self.t2 = 0
        self.t3 = 0
        self.t4 = 0
        self.t6 = 0
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
        # self.Obtain_Waypoints()
        self.get_mission_coordinates()
        self.obtain_current_wayPoint_ned_gps = threading.Thread(target = self.Obtain_Current_Waypoint_recent).start()

        
        #self.scheduledTaskList.append(ScheduleTask(0.02, self.lidar.update_sitl_sensor))  # 0.02
        #self.scheduledTaskList.append(ScheduleTask(0.000000000000000001, self.lidar.give_scan_values))
        #self.give_scan_values()
        # self.scheduledTaskList.append(ScheduleTask(0.00002,self.lidar.update_rplidar))
        t2 = threading.Thread(target=self.lidar.update_sitl_sensor)#/*/* returns a list in self.raw_data
        #stores data to self.lidar.raw_data to 64 elements ordered.
        t2.start() #/*/*check
       
       
        #self.scheduledTaskList.append(ScheduleTask(0.1,self.check_sensor_Raw))
       
        # t3 = threading.Thread(target=self.Obtain_Current_Waypoint)
        # t3.start()

        print("All threads started!")
        print("Starting The Front sensor sensor in 10 seconds")
        time.sleep(10)
        #self.check_sensor_Raw()

        

           
#   takes in direction vector and a map in global coordinates to give minimum distance to obstacle in that sector, returns true if obstacle is present                        
        while True:
            self.update_mission()
    
    def Obtain_Current_Waypoint_recent(self):
        while True:
            try:
                self.current_waypoint=self.waypoints[self.currentWP]
                self.next_waypoint=self.waypoints[self.currentWP+1]
                self.i_current_waypoint = self.i_waypoints[self.currentWP]
                self.i_next_waypoint = self.i_waypoints[self.currentWP+1]
            except KeyboardInterrupt:
                print("Exception in obtain_Current_Waypoint")
    #!/usr/bin/env python


# #############iitb CODE HERE################
# # parameters
# theta = 0.001 #Angle about z axis
# err = 0.3 #0.3 #Position tolerance in meters
# wstar = 5 #0.8 #Safety distance W*

# # Drone controller parameters
# x_a=0
# y_a=0
# z_a=0
# psi_a=0

# # Laser parameters
# counter = 0

# global height_flag
# height_flag = True
# # rospy rate
# looprate=20

# # P controller linear and angular velocity
# kp_omega = 1 #10
# kp_lin = 1
# Vmax = 1 # maximum velocity of the robot
# Wmax = np.pi #np.pi/2 changed from +-90 to +- 180

# #Laser params
# laser_range_min = 0
# laser_range_max = np.inf
# laser_step = 0
# laser_ang_min = 0
# laser_ang_max = 0

# # Tangent bug parameters
# state = 0
# d_rob2goal = 0
# d_reach = 0
# d_followed = 0
# states = {0 : 'Motion to Goal', 1 : 'Boundary Following', 2 : 'Reached Goal!!!', 3 : 'Impossible to reach goal!'}
# tan_list = []
# last_motion_vec = np.array([1e-7, 1e-7])
# last_motion_ang = 0
# bound_pos = []
# check_loop = 0


    def get_mission_coordinates(self):#Gets the whole mission accurately, including takeoff and land if present
        try:
            coordinates = []
            coordinates_ned = []
            while True:
                try:
                    self.mavlinkInterface.mavConnection.waypoint_request_list_send()
                    msg = self.mavlinkInterface.mavConnection.recv_match(type=['MISSION_COUNT'],blocking=True,timeout=1.5)
                    self.waypoint_count = msg.count
                    print("Count Mission successful ")
                    break
                except Exception as e:
                    print("Retrying to obtain count",e)
                    time.sleep(1)
            i = 0
            home_lat = 0
            home_lon = 0
            home_alt = 0
            # Loop through all mission items
            mission_counts = self.waypoint_count
            print("Mission Count is ",mission_counts)
            while i < mission_counts:
                try:
                    # Request the mission item
                    #self.mavlinkInterface.mavConnection.mav.mission_request_int_send(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, i)
                    self.mavlinkInterface.mavConnection.waypoint_request_send(i)
                    self.mavlinkInterface.mavConnection.waypoint_request_list_send()
                    msg = self.mavlinkInterface.mavConnection.recv_match(type=['MISSION_ITEM'], blocking=True,timeout=1)
                    #Assuming that the takeoff command is at seq id 1, home position is assumed to be the point of takeoff!
                    if i==1:
                        home_lat = msg.x 
                        home_lon = msg.y 
                        home_alt = msg.z

                    if msg is not None:
                        # Check if the mission item is a navigation command
                        if msg.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
                            # Get the GPS coordinates of the waypoint
                            lat = msg.x 
                            lon = msg.y 
                            alt = msg.z

                            # Convert GPS coordinates to NED
                            d_lat = lat - home_lat
                            d_lon = lon - home_lon
                            d_alt = alt - home_alt
                            R = 6371000  # Earth's radius in meters
                            x = d_lat * math.pi / 180 * R
                            y = d_lon * math.pi / 180 * R * math.cos(home_lat * math.pi / 180)
                            z = d_alt

                            # Add the NED coordinates to the list
                            coordinates_ned.append((x, y, z))
                            coordinates.append((lat,lon,alt))
                        elif msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                            print(f"takeoff command at index {i}.")
                            coordinates.append("TakeOff")
                            coordinates_ned.append("TakeOff")
                        elif msg.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
                            print(f"land command at index {i}.")
                            coordinates.append("land")
                            coordinates_ned.append("land")
                        elif msg.command == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                            print(f"RTL command at index {i}.")
                            coordinates.append("RTL")
                            coordinates_ned.append("RTL")
                        else:
                            print(f"non-navigation command at index {i} with command value {msg.command}.")
                            coordinates.append("Non-Navigation Command")
                            coordinates_ned.append("Non-Navigation Command")
                        i+=1 #Increment i only if the mission item has been successfully retrieved
                except:
                    time.sleep(1)
                    pass
            print("The following are the coordinates obtained in lat/lon/alt and in NED!!")     
            print(coordinates)
            print(coordinates_ned)          
            self.waypoints = coordinates
            self.i_waypoints = coordinates_ned    
            self.homeLocation = coordinates[0]  #Assuming Home position is the takeoff Position     
            print('home location is',self.homeLocation)
            print('inertial waypoints', self.i_waypoints)
            print('mission waypoint is', self.i_waypoints[1])
        except Exception as e:
            print(e)



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
            print(global_map)
            obstacle_vector = [global_map[i][0]-self.px,global_map[i][1]-self.py]

            obstacle_vector_magnitude=self.vec.mag2d(obstacle_vector)
            print('direc_vec,',direc_vec)
            print('obstacle_vec',obstacle_vector)

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
           
            temp_wp_dist_vec=[self.i_current_waypoint[0]-self.px,self.i_current_waypoint[1]-self.py]
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
            self.guided_target= [(px*wp2*x1 - px*wp2*x2 - py*wp1*x1 + py*wp1*x2 + wp1*x1*y2 - wp1*x2*y1)/(wp2*x1 - wp2*x2 - wp1*y1 + wp1*y2),(px*wp2*y1 - px*wp2*y2 - py*wp1*y1 + py*wp1*y2 + wp2*x1*y2 - wp2*x2*y1)/(wp2*x1 - wp2*x2 - wp1*y1 + wp1*y2)]
    
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
            self.check_B()
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
                        print("Generated next waypoint was none, so skipped updating the current waypoint and next waypoint.")
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
        #give value to ranges variable here

        self.front_sensor.handle_raw_data() #returns x and y coordinates of only masked points

        self.update_vars() #for transferign variable values across classes
        self.coordinate_transform.x = self.front_sensor.X
        self.coordinate_transform.y = self.front_sensor.Y  
        #print(self.front_sensor.X)
       
        self.coordinate_transform.update_vehicle_states()#gets heading and rotation matrix
       
    
        self.heading_vector() #gives heading of drone based on waypoints
        
        #
        self.coordinate_transform.rotate_body_to_inertial_frame() #returns 3xn matrix containing n obstacles
        # Create Obstacle map out of obstacle vector
        self.navigation_controller.obstacle_map=None #fetch a new map everytime
        #print(self.coordinate_transform.obstacle_vector_inertial)
        if self.coordinate_transform.obstacle_vector_inertial.size != 0: #inertial map updation logic
            if(self.relativeAlt>1):
                self.navigation_stack() #self.matmap is world map matrix, in inertial coordinates
        self.count_dum=self.count_dum+1 
        
        self.local_fetcher() #fetches local 2d occupancy grid        
        self.local_mat_to_list() #converts into list[[1,2]] Gives inertial coordinates of nearby obstacles
        #self.np_mat_to_list=np.array(self.mat_list) #get obstacles in world frame as np array
            
        
        #comment this line below to use estimated map instead.
        
        self.np_mat_to_list=np.array([[0,-50]])
        temp=self.i_waypoints[0] 
        
        if(self.currentMode=='AUTO' and self.i_current_waypoint!='TakeOff'):#and self.i_current_waypoint!=0 and self.terrainAlt>2 and self.i_current_waypoint!="TakeOff"):
                #if something is in heading direction, within desired angle and engaging distance, then engage brake/switch to guided
                #print('inside loop')
                temp_wp_dist_vec=[self.i_current_waypoint[0]-self.px-temp[0],self.i_current_waypoint[1]-self.py-temp[1]] #used to find if the current vector to next waypoint i
                print('inside check loop')
                self.desired_angle=30
                self.engaging_distance=15
                if (self.vx*self.vx+self.vy*self.vy)>2:
                    bool_temp,self.dist_temp=self.obstacle_in_direction(self.wpvec,self.np_mat_to_list,self.desired_angle,self.engaging_distance) # if any obstacle on the way
                    #print('wpvec',self.wpvec,'bool',bool_temp,'distance',self.dist_temp)
                    if(bool_temp and self.vec.mag2d(temp_wp_dist_vec)>abs(self.dist_temp)): #if obstacle needs to be avoided
                    #print('vx,vy',self.vx,',',self.vy)
                                    
                    
                    #print('wpvec',self.wpvec,'dist',self.px,',',self.py,'dist is',self.dist_temp)
                    ##time.sleep(0.1)
                    #if(bool_temp and (self.vx*self.vx+self.vy*self.vy)>2):# and self.vec.mag2d(temp_wp_dist_vec)>abs(self.dist_temp)): #if obstacle needs to be avoided
                    #if(True):
                    
                        while(self.currentMode=='AUTO' and self.currentMode!='GUIDED'):
                            print('sent brake mode request')
                            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,4))
                            print('vx,vy',self.vx,',',self.vy)
                            
                            print("Braked with Guided Mode Logic")                        
                            time.sleep(0.1)

                            
        if(self.currentMode=='GUIDED'):
            print('means we have stopped')
            self.run(self.current_waypoint[0],self.current_waypoint[1])

        #self.np_mat_to_list=np.array([[-0,-30]])
        #assume that list is correct
       
        #todo instead of velocity use waypoint vector?

        #self.np_mat_to_list=np.array([[0,-149],[-5,-150],[5,-150],[0,-221],[0,-226],[-5,-221],[5,-226],[0,-297],[0,-304],[5,-297],[-5,-304]])
        #switching logic for auto

        if(np.size(self.np_mat_to_list)!=0):
                #todo handle minimum distance from obstacle
            self.basic_guided_switch_and_plan() #Only works in auto
                
                # Brake command
      
   
        
    def heading_vector(self):
        #todo separate logic for rtl
        try:
        
            self.wpvec=[self.vx,self.vy]
            #self.wpvec=[self.i_current_waypoint[0]-self.i_previous_waypoint[0],self.i_current_waypoint[1]-self.i_previous_waypoint[1]] # wpvec is direction of moving
            
        except:   
            print('cant have wpvec')    
            pass
            #self.wpvec= [self.i_current_waypoint[0],self.i_current_waypoint[1]]
            
            
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
             
        self.navigation_map.map=self.navigation_map.convert_rel_obstacle_to_inertial1(self.coordinate_transform.obstacle_vector_inertial.T[:,0:2])#inertial frame grid 
        #self.navigation_map.inertial_grid=np.unique(np.concatenate((self.navigation_map.map, self.navigation_map.inertial_grid),axis=0),axis=0)
        #Relative obstacle is sent to the navigation algorithm
        #**todo send this map to update world map, and extract obstacle map from it
        #add a variable for world_map      
        #print(self.navigation_map.map)
        #adds dummy navigation_map
        
        for dummat in self.navigation_map.map:
            
            self.matmap[int(np.round(self.field_y/2-dummat[0,1]))][int(np.round(dummat[0,0]+self.field_x/2))]=1 
       
        




    def local_fetcher(self):
        self.relmatmap=self.matmap[int(np.ceil(self.field_y/2-self.py-self.length_fetcher/2)):int(np.ceil(self.field_y/2-self.py+self.length_fetcher/2)),int(np.ceil(self.px+self.field_x/2-self.width_fetcher/2)):int(np.ceil(self.px+self.field_x/2+self.width_fetcher/2))]
        self.res=1
    def local_mat_to_list(self):
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


    def angle2goal(x_goal, y_goal):
        """Computes the angle to reach the goal in respect to world coords.
        Used to check if an obstacle is obstructing the straight path.
        """
        global x_a, y_a, z_a, psi_a
        ang = math.atan2(y_goal - y_a, x_goal - x_a) - theta
        ang = (ang + np.pi) % (2 * np.pi) - np.pi # get angle in [-pi, pi)

        return ang

    def range2cart(ranges, idx):
        """Returns cartesian coord of Scan measurement in respect to world
        frame.
        """
        global x_a, y_a, z_a, psi_a
        T0r = np.array([
                        [np.cos(theta), -np.sin(theta), 0, x_a],
                        [np.sin(theta), np.cos(theta), 0, y_a],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]
                    ])
        rangex_robot = ranges[idx] * np.cos(laser_step * idx + laser_ang_min)
        rangey_robot = ranges[idx] * np.sin(laser_step * idx + laser_ang_min)
        range_robot = np.array([rangex_robot, rangey_robot, 1, 1]).reshape(-1, 1)
        range_world = (np.matmul(T0r, range_robot)).ravel()

        return range_world[:2]

    #takes in waypyoints in north-east global coordinate frame, and gives lienar and angualr vel as output'

    def run(self,x_goal, y_goal):
        global counter
        global height_flag
        global x_a, y_a, z_a, psi_a, psi_o
        x_a=self.px
        y_a=self.py
        psi_a=self.yaw #yaw northing easting
        #psi_o is calculated in controller_2

        # pub_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        # Vel_cmd = Twist()

        # rospy.Subscriber('/mavros/local_position/pose', PoseStamped, drone_pose_cb, (x_goal, y_goal))
        # rospy.Subscriber('/laser/scan', LaserScan, callback_scan)
        # rate = rospy.Rate(looprate)
        
        v_x=0 # in world frame or body frame
        v_y=0
        a_z=0

        # while not rospy.is_shutdown():
        #     # bring robot to z height and hold
        #     v.z=np.clip(3*(z_cmd-z_a),-0.5,0.5)
        #     if (height_flag):
        #         while (z_cmd != round(z_a, 2)):
        #             v.z=np.clip(3*(z_cmd-z_a),-0.5,0.5)
        #             v.x=0
        #             v.y=0
        #             pub_vel.publish(Vel_cmd)
        #         height_flag = False


        #     rospy.loginfo(round(math.sqrt(math.pow(x_a - x_goal, 2) + math.pow(y_a - y_goal, 2))))

        if (round(math.sqrt(math.pow(x_a - x_goal, 2) + math.pow(y_a - y_goal, 2))) == 0.0):
            v_x=0
            v_y=0
            a_z=0
            #enter mavlink commands for setting velocity
            pub_vel.publish(Vel_cmd)
            
        else:
            print (states[state])
#s
            if np.linalg.norm([x_a - x_goal, y_a - y_goal]) < err and state != 2:
                change_state(2)

#mostly taken care by automission
            if state == 0:
                V, W = motion2goal(x_goal, y_goal)
                v_x = V*np.cos(psi_a)
                v_y = V*np.sin(psi_a)
                a_z = W
                #enter mavlink commands to send vloeicty and angular velocity
                pub_vel.publish(Vel_cmd)
            elif state == 1:
                V, W = boundary_following(x_goal, y_goal)
                v_x = V*np.cos(psi_o)
                v_y = V*np.sin(psi_o)
                a_z = W
        

    def find_continuities(ranges):
        """ Returns a list of continuities intervals (min, max) for each
        detected obstacle, i.e. the closed intervals of <ranges> indices
        where an obstacle is detected (a continuity).
        Params
        ------
        ranges: sensor_msgs.msg.LaserScan.ranges
            The ranges returned by LaserScan messages.

        Returns
        -------
        cont_lims: list
            A list of tuples that represent each continuity interval as
            (min, max), where the values correspond to the <ranges> indices
            where the condition is satisfied.
        """
        # Get indices where laser range < range_max
        cont_indx = np.nonzero(np.array(ranges) < laser_range_max)[0]
        # Get superior and inferior limits of continuities
        lim_sup = np.array([x for i, x in enumerate(cont_indx)
                            if (x + 1 != cont_indx[(i + 1) % len(cont_indx)])])
        lim_inf = np.array([x for i, x in enumerate(cont_indx)
                            if (x - 1 != cont_indx[(i - 1) % len(cont_indx)])])
        cont_lims = [x for x in zip(lim_inf, lim_sup) if x[0] != x[1]]

        return cont_lims

    def check_blocking_oi(cont_idx, x_goal, y_goal):
        #TODO check if robot will collide even when goal is visible
        """Checks if any Oi is blocking the path to goal by angle
        comparison.

        Params
        ------
        cont_idx: list
            A list of tuples that define a continuity region by means of
            LaserScan indices.
        x_goal: float
            The horizontal coordinate of the goal
        y_goal: float
            The vertical coordinate of the goal

        Returns
        -------
        blocking: boolean
            Indicates whether an Oi is blocking the path to goal
        reg_num: int or None
            The region number that is blocking the path to goal, i.e. the
            index of <cont_idx> that represents the blocking region. If
            <blocking> == False, this value is None
        """
        global x_a, y_a, z_a, psi_a
        ang2g = angle2goal(x_goal, y_goal)
        reg_num = None
        blocking = False

        for i, region in enumerate(cont_idx):
            lim_inf = laser_step * region[0] + laser_ang_min
            lim_sup = laser_step * region[1] + laser_ang_min
            if lim_inf <= ang2g <= lim_sup:
                oi_mat = get_oi_coord(list(region))
                oi_inf_dist = np.linalg.norm(oi_mat[0] - np.array([x_a, y_a]))
                oi_sup_dist = np.linalg.norm(oi_mat[1] - np.array([x_a, y_a]))
                if oi_inf_dist < d_rob2goal or oi_sup_dist < d_rob2goal:
                    blocking = True
                    reg_num = i
                    break

        return blocking, reg_num

    def get_oi_coord(cont_idx_list):
        """Returns a matrix with world cartesian coords of each obstacle.
        Params
        ------
        ranges: sensor_msgs.msg.LaserScan.ranges
            The ranges returned by LaserScan messages.
        cont_idx_list: list
            A list of each <ranges> indices that contains a Oi (endpoint).

        Returns
        -------
        oi_mat: np.array
            A matrix that contains each endpoint Oi coords as a row vector,
            i.e. [ [Oi_x1, Oi_x1], [Oi_x2, Oi_x2], ..., [Oi_xn, Oi_xn] ].
        """
        oi_mat = np.empty((len(cont_idx_list), 2))
        for i, x in enumerate(cont_idx_list):
            oi_mat[i, :] = range2cart(ranges_, int(x))

        return oi_mat

    def get_tangent_vector(region):
        """Calculates tangent vector based on the smoothed euclidean norm.

        Params
        ------
        region: list or tuple
            A list of tuples or a single tuple of discontinuites.

        Returns
        -------
        tangent: np.array
            Tangent vector to the closest obstacle point.
        closest_point
            The closest approximated point.
        """
        global x_a, y_a, z_a, psi_a
        reg_idx = []

        if isinstance(region, tuple):
            region = [region]
        for lims in region:
            lim_inf, lim_sup = lims
            idxs = np.unique(np.linspace(lim_inf, lim_sup, -(-3*(lim_sup - lim_inf) // 4) + 1, dtype=int))
            reg_idx = np.r_[np.array(reg_idx), idxs]

        oi_mat = get_oi_coord(reg_idx)
        pos_vec = np.array([x_a, y_a])
        norm_mat = np.linalg.norm(pos_vec - oi_mat, axis=1) ** 2
        min_idx = np.argmin(norm_mat)
        min_dist = norm_mat[min_idx]
        h = 0.1
        sum_, div_ = np.array([0.0, 0.0]), 0

        for i, a in enumerate(norm_mat):
            temp = np.exp((min_dist - a) / (2 * h ** 2))
            sum_ += temp * (pos_vec - oi_mat[i, :])
            div_ += temp

        rot90 = np.array([
                [np.cos(last_motion_ang), -np.sin(last_motion_ang)],
                [np.sin(last_motion_ang), np.cos(last_motion_ang)]
            ])
        D = sum_ / div_
        tangent = (np.matmul(rot90, D.reshape(-1, 1))).ravel()
        closest_point = pos_vec - D

        return tangent, closest_point

    def choose_oi(ranges, cont_idx, x_goal, y_goal):
        """Returns the coordinates of best Oi to follow if the robot is in
        motion to goal behavior AND there is no obstacles blocking the goal
        otherwise, it returns a velocity vector
        Params
        ------
        ranges: sensor_msgs.msg.LaserScan.ranges
            The ranges returned by LaserScan messages.
        cont_idx: list
            A list of tuples that define a continuity region by means of
            LaserScan indices.
        x_goal: float
            The horizontal coordinate of the goal.
        y_goal: float
            The vertical coordinate of the goal.

        Returns
        -------
        oi2follow_coord: np.array
            An array with the followed Oi coordinates or velocity vector.
        """
        global x_a, y_a, z_a, psi_a
        global d_reach, d_followed
        pos_vec = np.array([x_a, y_a])
        goal_vec = np.array([x_goal, y_goal])

        if isinstance(cont_idx, tuple):
            cont_idx_list = list(cont_idx)
        else:
            cont_idx_list = [x for t in cont_idx for x in t]

        oi_mat = get_oi_coord(cont_idx_list)
        dist_pos2oi = np.linalg.norm((pos_vec - oi_mat), axis=1)
        dist_oi2goal = np.linalg.norm((goal_vec - oi_mat), axis=1)
        heuristic = dist_pos2oi + dist_oi2goal

        if state == 1:
            tangent, closest_point = get_tangent_vector(cont_idx)
            safe_oi = safety_distance(closest_point, tangent)
            d_reach = np.linalg.norm(goal_vec - closest_point)
            oi2follow_coord = safe_oi

        else:
            tangent, closest_point = get_tangent_vector(cont_idx)
            safe_oi = safety_distance(closest_point, tangent)
            d_reach = np.linalg.norm(goal_vec - closest_point)
            oi2follow_coord = safe_oi

        if d_reach <= d_followed:
            d_followed = d_reach

        return oi2follow_coord

# 
    def change_state(n):
        global state, check_loop, bound_pos
        state = n
        check_loop = 0
        bound_pos = []
        print (states[state])
        #print({}.format(states[state]))

    def safety_distance(oi_coord, tangent_vec):
        """Adds a safety distance to followed Oi.
        """
        global x_a, y_a, z_a, psi_a
        vec_robot2oi = oi_coord - [x_a, y_a] # direction
        oi_dist = np.linalg.norm(vec_robot2oi) # euclidean distance between 2 points (closet point, current position)

        if oi_dist == 0:
            oi_dist = 1e-3

        oi_norm = vec_robot2oi / oi_dist # unit vector
        if oi_dist > 2*wstar: # was 1.3
            #print ("condition 1", oi_dist, 2.3*wstar)
            alpha = 1
            beta = 1
        else:
            #print ("condition 2", oi_dist, 2.3*wstar)
            alpha = 3 #3psi_d
            beta = 1

        oi_safe = beta * (vec_robot2oi - (wstar * oi_norm)) + alpha * tangent_vec
        #print ("here", vec_robot2oi, oi_norm)
        return oi_safe

# def get_laser_params(data):
#angle_max



#     """Sets global laser parameters."""
#     global laser_range_min, laser_range_max, laser_step, laser_ang_min
#     global laser_ang_max
#     laser_range_min = data.range_min
#     laser_range_max = data.range_max
#     laser_step = data.angle_increment
#     laser_ang_min = data.angle_min
#     laser_ang_max = data.angle_max

# def callback_scan(data):

#ranges min to max angle ranges, take lidar, counter is not relevant 
#     global ranges_, counter
#     if counter < 10:
#         get_laser_params(data)
#     ranges_ = data.ranges
#     #rospy.loginfo(set(ranges_))

##add current waypoint here?
##is it yaw?
def drone_pose_cb(self):
    global x_a, y_a, z_a, psi_a, d_rob2goal

    x_a = self.px #+x_a_offset[droneid-1]
    y_a = self.py #+y_a_offset[droneid-1]
    z_a = self.pz
#in rads
    psi_a = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2**2+q3**2))
    pos_vec = np.array([x_a, y_a])
# enter 

    goal_vec = np.array([args[0], args[1]])
    d_rob2goal = np.linalg.norm(pos_vec - goal_vec)

#what is frame of these variables
    def controller(x_goal, y_goal):
        global x_a, y_a, z_a, psi_a
        psi_d = np.arctan2((y_goal - y_a), (x_goal - x_a))
        psi_e = psi_d - psi_a
        psi_e = np.arctan2(np.sin(psi_e),np.cos(psi_e))
        omega = kp_omega * psi_e
        omega = np.clip(omega, -1*Wmax, Wmax)

        d = math.sqrt(pow((x_goal-x_a),2) + pow((y_goal-y_a),2))
        v_lin = kp_lin * d
        v_lin = np.clip(v_lin, -1*Vmax, Vmax)
        print ("GOAL:", x_goal, y_goal)
        return v_lin, omega

    def controller2(x_dir, y_dir):
        global x_a, y_a, z_a, psi_a, psi_o
        psi_o = np.arctan2((y_dir), (x_dir))
        psi_e = 0 - psi_a
        psi_e = np.arctan2(np.sin(psi_e),np.cos(psi_e))
        omega = kp_omega * psi_e

        omega = np.clip(omega, -1*Wmax, Wmax)
        #print ("GOAL:", omega, Wmax )
        d = math.sqrt(pow((x_goal-x_a),2) + pow((y_goal-y_a),2))
        v_lin = 1
        v_lin = np.clip(v_lin, -1*Vmax, Vmax)
        return v_lin, omega

    def motion2goal(x_goal, y_goal):
        """Executes motion to goal behaviour using feedback velocity
        controller with feedback linearization.
        Params
        ------
        x_goal: float
            The horizontal coordinate of the goal.
        y_goal: float
            The vertical coordinate of the goal.

        Returns
        -------
        v: float
            The linear velocity given by feedback linearization
        w: float
            The angular velocity given by feedback linearization
        """
        #TODO improve blocking check
        global d_followed, tan_list, last_motion_ang, bound_pos
        global x_a, y_a, z_a, psi_a
        cont_idx = find_continuities(ranges_)
        blocking, reg_num = check_blocking_oi(cont_idx, x_goal, y_goal)

        if blocking:
            d_reach_old = d_reach
            oi = choose_oi(ranges_, cont_idx[reg_num], x_goal, y_goal)
            oi_safe = oi
            v, w = controller(oi_safe[0], oi_safe[1])

            if np.linalg.norm(([x_goal, y_goal] - oi_safe)) > d_rob2goal:
                d_followed = d_reach_old
                tan_list = []
                change_state(1)
        else:
            v, w = controller(x_goal, y_goal)
            oi_safe = np.array([x_goal, y_goal])

        last_motion_vec = [x_a, y_a] - oi_safe
        norm_vec = last_motion_vec / np.linalg.norm(last_motion_vec)
        dot_prod = np.dot(norm_vec, np.array([1, 0]))
        if np.sign(np.arctan(dot_prod) + 2*np.pi) == 1:
            last_motion_ang = np.pi/2
        else:
            last_motion_ang = -np.pi/2
        return v, w

    def boundary_following(x_goal, y_goal):
        global d_followed, tan_list, check_loop, bound_pos
        global x_a, y_a, z_a, psi_a
        #print ("here--------------------------")
        cont_lims = find_continuities(ranges_)
        #print (cont_lims)
        if not cont_lims:
            change_state(0)
            return 0, 0
        pos_vec = np.array([x_a, y_a])

        if bound_pos:
            bound_dists = np.linalg.norm(pos_vec - np.array(bound_pos), axis=1)
            '''
            if not check_loop and np.max(bound_dists) > err:
                check_loop = 1
            elif check_loop and any(bound_dists[:-15] <= err):
                change_state(3)
                return 0, 0
            '''
        closest_oi = choose_oi(ranges_, cont_lims, x_goal, y_goal)
        oi = closest_oi
        bound_pos.append([x_a, y_a])
        x_new = oi[0]
        y_new = oi[1]
        v, w = controller2(x_new, y_new)
        #print ("boundary following: d states", d_reach, d_followed)
        if d_reach <= d_followed:
            tan_list = []
            change_state(0)
            bound_pos = np.array([x_a, y_a])

        return v, w