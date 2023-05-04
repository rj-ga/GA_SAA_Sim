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
import util.PrasRPLidar as prasLidar
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
        # self.lock = threading.Lock()
        self.handleRecievedMsgThread = None

        #Initialise SITL driver
        #self.lidar = driver.SensorDriver('SITL')0,0,2,12
        self.lidar = prasLidar.PrasRPLidar_(4,-90,2,10)#scanMode, OffsetAngle, MinRange and MaxRange
        #Connect to the listener - ensure the listener is running in background!!
        #self.lidar.connect_and_fetch()
        print('init')
        #Front sensor
        self.front_sensor = estimation.Sensor(1,math.pi/180,10,2,0)#maxRange minRange and Offset. I have already handled offset.
        #SITL
        #self.front_sensor = estimation.Sensor(1,0.03098,40,1,-0.976)  

        #Initialise pre processor
        self.coordinate_transform = estimation.DataPreProcessor()

        #initialise navigation controller
        self.navigation_controller = control.ObstacleAvoidance(max_obs=20+3)

        self.navigation_map = estimation.DataPostProcessor()

        #Initialize vector class of Vectormath
        self.vec = vmath.vector()

        ##########################################  Find out how to take out home and waypoint location via Mavlink

        #Initialize Planner class
        self.plan = planner.Planner()

        # Home posiiton
        self.home = np.array([])

        # Waypoint position 
        self.wp = np.array([])

        #Brake
        self.brake = 0
        self.alreadybraked = 0
        self.initvar=1

        #Initiatialising Position Predictor Vectors
        self.prev_px = 0
        self.prev_py = 0
        self.px = 0
        self.py = 0
        
        self.count_dum=0
        self.t11=[]
        self.t12=[]
        self.t13=[]
        self.t14=[]
        self.t1_dum=0
        self.count_iter=0

        # Terminate trigger
        self.terminate = 0    
        
        # Pilot overriding
        self.overriding = 1    # Initially pilot control

        self.t2 = 0
        self.t3 = 0
        self.t4 = 0
        self.t6 = 0

        self.g = 0
        self.v = 0
        self.t = 0
        self.ti = 0
        self.tt1 = 0

        self.A_reached = 0
        self.B_reached = 0
        self.C_reached = 0

        self.obstacle_map_optim = np.array([[]])
        
        self.current_waypoint = 0
        self.next_waypoint = 0
        self.waypoint_count = 0
        self.nextwaypoint = 0
        self.waypoints = []
        self.i_waypoints =  []
        self.i_current_waypoint = 0
        self.i_next_waypoint = 0
        self.sensor_trig = 0 #Use this as an obstacle avoidance triggerer
        self.field_x=1500
        self.field_y=1500
        self.checked=1
        
        self.log_file = None
        self.csv_file = None

        self.matmap=np.array([[0]*self.field_x]*self.field_y)
        np.reshape(self.matmap,(self.field_x,self.field_y))
        self.width_fetcher=100
        self.length_fetcher=100
        self.relmatmap=np.array([[0]*self.width_fetcher]*self.length_fetcher)
        
    
    def init(self):
        super().init()

        # set data stream rate
        self.set_data_stream()
        
        # start our recieving message handling loop
        self.handleRecievedMsgThread = threading.Thread(target=self.handle_recieved_message)
        self.handleRecievedMsgThread.start()

        ### Starting reading threads as they are while loops ###
        #t1 = threading.Thread(target=self.lidar.give_scan_values)
        #t1.start()
        #t2 = threading.Thread(target=self.lidar.read_fast)
        #t2.start()

        
        # set data stream rate
        self.set_data_stream()
        
        # start our recieving message handling loop

        #Scheduled the threads
        print("Executing all threads")
        print("Fetchin all the waypoints")
        #self.Obtain_Waypoints()
        time.sleep(2)
        t = threading.Thread(target=self.lidar.simple_express_scan)
        t.start()
        #self.scheduledTaskList.append(ScheduleTask(0.02, self.lidar.update_sitl_sensor))  # 0.02
        #self.scheduledTaskList.append(ScheduleTask(0.000000000000000001, self.lidar.give_scan_values))
        #self.give_scan_values()
        #self.scheduledTaskList.append(ScheduleTask(0.00002,self.lidar.update_rplidar))
        #self.scheduledTaskList.append(ScheduleTask(0.08,self.update_vars))  # 0.09
        #self.scheduledTaskList.append(ScheduleTask(0.05,self.front_sensor.handle_raw_data))  # 0.05
        #self.scheduledTaskList.append(ScheduleTask(0.3,self.update_mission)) # 0.45 works works very good without SAAPlanner functions  |  0.35 works very well with SAAPlanner but without Guided naviagtion, maneuver and planned_maneuver logics
        """_summary_
        """

        
        self.world_map = []
        tprint = threading.Thread(target=self.printing)
        tprint.start()



        self.create_csv_file()

        #self.world_map = [(10,-2),(8,-16),(18,-27),(36,-36),(-1,-5),(-12,-8),(-8,2),(-6,-25),(-19,-17),(-30,-25),(-16,-35),(-22,-7),(2,-41),(3,-21),(0,-2),[0,-18],(0,-150),(0,-225),(0,-300),(0,-375),(0,-450),(0,-525),(0,-25)]
       
        
        # t3 = threading.Thread(target=self.Obtain_Current_Waypoint)
        # t3.start()
        # print("All threads started!")
        #self.check_sensor_Raw()
        #t4 = threading.Thread(target=self.check_sensor_Raw)
        #t4.start()
        time.sleep(15)
        #t4 = threading.Thread(target=self.update_mission)
        #t4.start()
        while True:

            self.update_mission()
             #self.DijKstras_Triggerer(self.world_map)
            #self.Testing_Waypoint()
#         while True:
#             if self.navigation_controller.obstacle_map is None:
#                 pass
#             else:
#                 time.sleep(0.01)

    def get_gps_coordinates(self):
        while True:
            try:
                msg = self.mavlinkInterface.mavConnection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                if msg is not None:
                    self.lat = msg.lat / 1e7
                    self.lon = msg.lon / 1e7
                    print(f"Latitude: {self.lat}, Longitude: {self.lon}")
                    break
            except:
                print("GPS_DATA_ERROR")

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
        for i in range(0,self.waypoint_count,1):
            local_coord=pm.geodetic2enu(self.homeLocation[0],self.homeLocation[1],100,self.waypoints[i][0],self.waypoints[i][1],100)
            self.i_waypoints.append((local_coord[1],-local_coord[0])) #/*/*change for real drone
        print(self.i_waypoints)


    def handle_mission_current(self,msg,nextwaypoint):
        #/*/*check
        if msg.seq > nextwaypoint:
            c_point_index = int(msg.seq)
            print(c_point_index)
            print ("Moving to waypoint ",c_point_index,self.waypoints[c_point_index])
            self.current_waypoint = self.waypoints[c_point_index]
            self.i_current_waypoint = self.i_waypoints[c_point_index]
                        
            nextwaypoint = msg.seq+1
            if(nextwaypoint<len(self.waypoints)):
                self.next_waypoint = self.waypoints[nextwaypoint]
                self.i_next_waypoint = self.i_waypoints[nextwaypoint]
                print ("Next Waypoint ", self.next_waypoint)
                return nextwaypoint
            else:
                print("Last Waypoint Reaching")
                return -1

    def Obtain_Current_Waypoint(self):
        nextwaypoint=0
        while True:
            #/*/*check
            msg = self.mavlinkInterface.mavConnection.recv_match(type=['GLOBAL_POSITION_INT', 'MISSION_CURRENT', 'MISSION_COUNT', 'HEARTBEAT'],blocking=True,timeout=1.5)
            self.mavlinkInterface.mavConnection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 192, 0, 4)
            if(msg!=None and msg.get_type()!=None):
                try:
                    if nextwaypoint==0:
                        self.current_waypoint=self.waypoints[1]
                        self.next_waypoint=self.waypoints[2]
                        self.i_current_waypoint = self.i_waypoints[1]
                        self.i_next_waypoint = self.i_waypoints[2]
                    if msg.get_type() == 'HEARTBEAT':
                        self.mavlinkInterface.mavConnection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 192, 0, 4)
                    if msg.get_type() == 'GLOBAL_POSITION_INT':
                        relative_alt = msg.relative_alt
                    if msg.get_type() == 'MISSION_CURRENT':
                        nextwaypoint = self.handle_mission_current(msg,nextwaypoint)
                        print ('Next Waypoint', nextwaypoint,self.next_waypoint)
                        print ("Current Waypoint",self.current_waypoint)
                        if nextwaypoint == None:
                            nextwaypoint = 0 #Need to handle None type
                        if nextwaypoint >= self.waypoint_count - 1:
                            if relative_alt<=1*1000*0.05: 
                                print ("Reached land altitude")
                                break
                    time.sleep(0.1)
                    #/*/*check
                    #self.mavlinkInterface.mavConnection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 192, 0, 4) # send heart beat to airframe per 1 sec

                except KeyboardInterrupt:
                    print("Exception in obtain_Current_Waypoint")

    def DijKstras_Triggerer(self,world_map): #This needs to be called everytime. Need to set a trigger such that I can call it after it has completed its execution Part
        #Fetch the local Obstacle list and read sensor Data Here:
        #self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), 2, -41, -10, 0, 0, 0, 0, 0, 0, 0, 0))

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
                if((abs(c[0]-self.px)<=7 and abs(c[1]-self.py)<=7) or self.sensor_trig==1):
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
                        print('Completed Dijtraks Maneavuer, switched to auto')
                    else:
                        pass
                        #print("The Drone was not in Auto mode to start Obstacle Avoidance")
                    break

    def Testing_Waypoint(self): #This is just for me to obtain testing waypoints
        c = [(2,0)]
        #c = self.i_waypoints
        for i in c:
            while((self.px-i[0])*(self.px-i[0])+(self.py-i[1])*(self.py-i[1])>=1):
                if(self.currentMode=='GUIDED'):#In Real World, z+ for fly up, in gazebo z axis is inverted
                    self.mavlinkInterface.mavConnection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message( 10, self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), i[0], i[1], 3, 0, 0, 0, 0, 0, 0, 0, 0))
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
        while self.currentMode != 'GUIDED':
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system, 
                                                                                                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                                                        4))
            print("switched To Guided")

    def AutoMode(self):
        while self.currentMode != 'AUTO':
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system, 
                                                                                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                                           3))
            print("switched To Auto")


    def check_sensor_Raw(self):  #Triggers avoidance if the front portion of the sensor detects obstacle under 5m
        while True:
            try:
                c = 0
                #/*/*add heading direction
                print(self.front_sensor.X[90])
                for i in self.front_sensor.data[85:95]:
                   c+=1
                   if i<5:#CHECK TRIGGER IN REAL TIME, WORKS AT TIMES AND DOES NOT AT TIMES LOL
                       self.sensor_trig=1
                       print("Trig",i,c)
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
        if self.initvar==1:
            print("Handbrake Initialize")
            self.initvar = 0
        #Only brake when previously brake command is not given, altitude is greater than 1
        if self.brake and not self.alreadybraked and self.relativeAlt>1:
            print("Switching to Brake Mode")
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                                           17))
            self.alreadybraked = 1
            self.t2 = time.time()
        self.check_mode(False)     # All required trigger switching in check_mode() is instantaneously done thru here (default - False)

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
        #self.front_sensor.data = self.lidar.raw_data[::-1]#Made these changes so that the actual sensor works like gazebo world
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
    def printing(self):
        while(True):
            print('matrix',self.world_map)
            time.sleep(10)
            
    def create_csv_file(self):
        self.log_time = time.time()
        self.csv_file = open(str(self.log_time)+'.csv','w')

    def log_data(self):
        list_raw_data_to_string = ','.join([str(elem) for elem in self.front_sensor.data])+",RAW_LIDAR_DATA_ENDS_HERE AND ROLL_PITCH_YAW BEGINS,"
        roll_pitch_yaw_string = str(self.roll)+','+str(self.pitch)+','+str(self.yaw)+",self.px and self.py BEGINS here,"
        selfpx_and_selfpy = str(self.px)+','+str(self.py)+',Now The GPS Lat and Lon Data,'
        gps_lat_lon_of_drone = '('+str(self.lat)+','+str(self.lon)+')'
        
        
        final_data = list_raw_data_to_string + roll_pitch_yaw_string + selfpx_and_selfpy + gps_lat_lon_of_drone+'\n'
        self.csv_file.write(final_data)
        if time.time()-self.log_time>=30:
            self.csv_file.close()
    
    
    def update_mission(self):
        while True:
            
            
            self.front_sensor.data = self.lidar.raw_data[::-1]
            self.get_gps_coordinates()
            self.log_data()

            #returns x and y coordinates of only masked points
            # print("FRONT",self.front_sensor.data[80:100])
            # print("RIGHT",self.front_sensor.data[350:359],self.front_sensor.data[0:10])
            # print("LEFT",self.front_sensor.data[170:190])
            # print("BACK",self.front_sensor.data[260:280])
            self.front_sensor.handle_raw_data()
            self.update_vars()
            self.coordinate_transform.x = self.front_sensor.X
            self.coordinate_transform.y = self.front_sensor.Y
            #print(self.lidar.raw_data[0:30])
            time.sleep(1)
            self.coordinate_transform.update_vehicle_states()#gets heading and rotation matrix
            self.count_iter=self.count_iter+1
            
            ## Updating states
            # Update all the related states and variables
            
            # Predict the pos_vector
            self.navigation_controller.predict_pos_vector() #just updates direction of heading in short dr
            # Raw data handling 
            
            # Update x,y used for datapreprocessing using updated front sensor data
            
            #print(np.unique(self.front_sensor.X)

            ## Building necessary map and variabes based on updated statesco
            
            # Calculate obstacle vectors 
            self.coordinate_transform.convert_body_to_inertial_frame() #returns 3xn matrix containing n obstacles
            # Create Obstacle map out of obstacle vector
            self.navigation_controller.obstacle_map=None #fetch a new map everytime
            #print(self.coordinate_transform.obstacle_vector_inertial)
            #print(self.coordinate_transform.obstacle_vector_inertial)
            if self.coordinate_transform.obstacle_vector_inertial.size != 0: #inertial map updation logic
                #if (self.relativeAlt>1):
                self.navigation_stack()

            self.local_fetcher()
            self.local_mat_to_list()

            self.np_mat_to_list = np.array(self.mat_list)
            self.world_map = self.np_mat_to_list
            
            # if len(self.front_sensor.X)>10:
            #     #print(self.front_sensor.X[80:110])
            #     pass
            # else:
            #     print("Error in length of front sensor")
            # time.sleep(1)

            #time.sleep(1)

            #take world map and convert to relative distance map
            if(self.navigation_map.world_map.size!=0):
                pass ###Basic Guided Switch and Plan
            #self.navigation_controller.obstacle_map = self.navigation_map.convert_inertial_to_rel_world()
            #$print(self.np_mat_to_list)

        
        
        
        # Minimizing obstacle points for same obstacle by averaging for nearby points       
        #self.map_optimize()
        # self.obstacle_map_optim=self.navigation_map.map
        # self.navigation_controller.obstacle_inertial=self.navigation_map.map
        # self.navigation_controller.obstacle_body=self.navigation_controller.obstacle_map

        # self.t11.append(round(time.time()-self.t1_dum,4))
        # if(self.count_dum%100==0):
        #     print('\n',self.t11)
        #     t11=[]
        #     print('\n',self.navigation_map.world_map)
        #     time.sleep(100)
            #with open('navigation_map.txt', 'a') as f:
               # f.write(str(self.navigation_map.world_map))
            #print('\n',self.navigation_map.world_map)

        



       
        

            
        # if self.currentMode == 'AUTO':
        #     ## Obstacle detection
        #     # Detection for Brake Command
        #     self.navigation_controller.basic_stop()
        #     # Brake command
        #     self.handbrake()

    #     self.check_mode(False)
        
    #     # Relavant mode conversions and Triggers
    #     self.trigger_avoidance()
    #     # if self.navigation_controller.guide and self.navigation_controller.guiding and self.brake:
    #     #     self.brake = 0
        
    #     self.t1_dum=time.time()

    #     ## Obstacle Avoidance section
    #     if self.navigation_controller.guiding and not self.plan.plan_command: 
    #         t1 = time.time()
    #         while np.all(np.array(self.lidar.raw_data) == 40):                  
    #             time.sleep(0.001)
    #             #print('In this loop')
    #             if not self.navigation_controller.guiding:
    #                 break
    #             # This is just to avoid being caught in this loop forever when lidar data is empty and guiding is True
    #             if self.vec.mag2d([self.vx, self.vy]) <= 0.08:
    #                 print('Drone is not moving')
    #                 print(f'guiding --> {self.navigation_controller.guiding}  avoided --> {self.navigation_controller.avoided} avoiding --> {self.navigation_controller.avoiding}')
    #                 break
    #             t2 = time.time()count+=1
    #             # If guiding True but evaded obstacle quickly, then just wait for 1 seconds to see any other obstacle after which break
    #             # Uncomment below if using exploratory method
    #             # if (t2-t1) >= 1:
    #             #     self.front_sensor.data = self.lidar.raw_data
    #             #     self.front_sensor.handle_raw_data()
    #             #     self.coordinate_transform.x = self.front_sensor.X
    #             #     self.coordinate_transform.y = self.front_sensor.Y
    #             #     self.coordinate_transform.convert_body_to_inertial_frame()
    #             #     self.navigation_stack()
    #             #     break
    #             if (t2-t1) >= 1:
    #                 self.navigation_controller.auto = 1

            
    #         if not np.all(np.array(self.lidar.raw_data) == 40):
    #             self.front_sensor.data = self.lidar.raw_data
    #             self.front_sensor.handle_raw_data()
    #             self.coordinate_transform.x = self.front_sensor.X
    #             self.coordinate_transform.y = self.front_sensor.Y
    #             self.coordinate_transform.convert_body_to_inertial_frame()

    #             if self.coordinate_transform.obstacle_vector_inertial.size != 0:
    #                 self.navigation_stack()

    #     '''For Exploratory Obstacle detection and avoidance (uses Guided navigation logic in SAAController)'''
    #     # #Guided navigation function   
    #     # self.navigation_controller.Guided_navigation()
    #     # self.trigger_avoidance()
    #     # #Maneuvering navigation function
    #     # self.maneuver()
    #     '''########################################################'''
    #     # self.t12.append(round(time.time()-self.t1_dum,4))
    #     # self.t1_dum=time.time()
    #     '''For Path Planned Obstacle Avoidance (uses Guided_waypoint logic in SAAPlanner)'''
    #     self.plan.make_wp_origin_center()
    #     if self.navigation_controller.guiding and self.t==0:
    #         self.t = 1
    #         print('Path Planning')
    #         self.plan.Pathplanning(self.navigation_controller.obstacle_map,False)
    #     else:
    #         print('No path planning')
    #         print(self.plan.c_wp_vec)
    #     # print(self.plan.c_wp_vec)
    #     # print(f'home --> {self.plan.home_pos}  waypoint --> {self.plan.w_vector}')
    #     # print(f'Target pos : {self.navigation_controller.target}   Current pos : {[self.px, self.py]}')
    #     '''########################################################'''
    #     # self.t13.append(round(time.time()-self.t1_dum,4))
    #     # self.t1_dum=time.time()
    #     self.planned_maneuver()
        
    #     # Store previous position 
    #    # self.previous_position_storer()
    #     # self.t14.append(round(time.time()-self.t1_dum,4))
    #     # self.count_dum=self.count_dum+1
    #     # if(self.count_dum>400):
    #     #     print("\nt11", self.t11, '\nt12', self.t12,'\nt13', self.t13,'\nt14', self.t14)
    #     #     self.count_dum=0
    #     #     self.t11=[]
    #     #     self.t12=[]
    #     #     self.t13=[]
    #     #     self.t14=[]

    #     '''Computational time debugging purpose'''
    #     # t2 = time.time()
    #     # if (t2-t1) >= self.ti:
    #     #     self.t = 1
    #     #     self.ti = t2-t1
    #     # print(f'           Notttttttttttttttttttttttttttttttttttttt good  {self.ti}')
    #     #print(f'                                                 time --> {t2-t1}')
    #     #print(f'Raw data Lidar --->  {np.unique(self.front_sensor.data)}')
    
    def local_fetcher(self):
        self.relmatmap=self.matmap[int(np.ceil(self.field_y/2-self.py-self.length_fetcher/2)):int(np.ceil(self.field_y/2-self.py+self.length_fetcher/2)),int(np.ceil(self.px+self.field_x/2-self.width_fetcher/2)):int(np.ceil(self.px+self.field_x/2+self.width_fetcher/2))]
        self.res=1
        
    def local_mat_to_list(self):
        self.mat_list=[]
        for i in range(0,len(self.relmatmap)):#no of rows
            for j in range(0,len(self.relmatmap[0])):
                if (self.relmatmap[i][j]==1):
                    self.mat_list.append([-self.width_fetcher/2+j*self.res+self.px,self.length_fetcher/2-i*self.res+self.py])
                          
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
        """Update vars function was getting filled....
        I am making another thread. 
        """
        try:
        #Angular inertial obstacle vector to grid based reading and that is stored in global map
        #self.navigation_map.convert_rel_obstacle_to_inertial(self.navigation_map.grid(self.coordinate_transform.obstacle_vector_inertial.T))
         #updates map in world frame
            self.navigation_map.map=self.navigation_map.convert_rel_obstacle_to_inertial1(self.coordinate_transform.obstacle_vector_inertial.T[:,0:2])#inertial frame grid 
            #print(self.navigation_map.map)
            for dummat in self.navigation_map.map:    
                #print(dummat)        
                self.matmap[int(np.round(self.field_y/2-dummat[0,1]))][int(np.round(dummat[0,0]+self.field_x/2))]=1 

        except AssertionError as e:
            print("ERROR",e)
            try :
                self.matmap[int(np.round(self.field_y/2-dummat[1]))][int(np.round(dummat[0]+self.field_x/2))]=1 
            except:
                print("Error Again!!")
#todo figure out why this happening

        # self.navigation_map.map=self.navigation_map.grid(self.navigation_map.convert_rel_obstacle_to_inertial1(self.coordinate_transform.obstacle_vector_inertial.T[:,0:2]))#inertial frame grid 
        # #self.navigation_map.inertial_grid=np.unique(np.concatenate((self.navigation_map.map, self.navigation_map.inertial_grid),axis=0),axis=0)
        # #Relative obstacle is sent to the navigation algorithm
        # #**todo send this map to update world map, and extract obstacle map from it
        # #add a variable for world_map
        # if(self.navigation_map.world_map.size!=0):
        #     self.navigation_map.world_map=np.unique(np.concatenate((self.navigation_map.world_map,self.navigation_map.map),axis=0),axis=0) ##Todo check uniqueness before adding.
        # else:
        #     self.navigation_map.world_map=self.navigation_map.map
        #print(self.navigation_map.world_map)
        #print("\n")
        
        #self.obstacle_storing_map()
        #print(f'Obstacle Inertial: {self.navigation_controller.obstacle_inertial}')

         
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
