# -*- coding: utf-8 -*-
"""
Created on Wed Jul  8 14:07:56 2020

@author: Sachchit Vekaria
@Organization: General Aeronautics Pvt Ltd
"""

# import necessary modules
from pickle import NONE
import time
from unittest import FunctionTestCase
from util.gacommonutil import CompanionComputer, mavutil, ScheduleTask
import threading
import logging


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

        #Front sensor
        #self.front_sensor = estimation.Sensor(1,1*math.pi/180,12,0.01,0)
        #SITL
        self.front_sensor = estimation.Sensor(1,0.03098,40,1,-0.976)  

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
        #self.set_data_stream()
        
        # start our recieving message handling loop
        #self.handleRecievedMsgThread = threading.Thread(target=self.handle_recieved_message)
        #self.handleRecievedMsgThread.start()

        #Scheduled the threads
        print("Executing all threads")
        self.scheduledTaskList.append(ScheduleTask(0.02, self.lidar.update_sitl_sensor))  # 0.02
        #self.scheduledTaskList.append(ScheduleTask(0.000000000000000001, self.lidar.give_scan_values))
        #self.give_scan_values()
        # self.scheduledTaskList.append(ScheduleTask(0.00002,self.lidar.update_rplidar))
        self.scheduledTaskList.append(ScheduleTask(0.08,self.update_vars))  # 0.09
        #self.scheduledTaskList.append(ScheduleTask(0.05,self.front_sensor.handle_raw_data))  # 0.05
        self.scheduledTaskList.append(ScheduleTask(0.3,self.update_mission)) # 0.45 works works very good without SAAPlanner functions  |  0.35 works very well with SAAPlanner but without Guided naviagtion, maneuver and planned_maneuver logics
        """_summary_
        """

        while True:
            if self.navigation_controller.obstacle_map is None:
                pass
            else:
                time.sleep(0.01)
            

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


    def update_mission(self):
        self.t1_dum=time.time()
        self.count_dum=self.count_dum+1
        
        if(self.count_dum==1):
            print(self.px,self.py)
        self.update_vars()  
       # print(self.front_sensor.data) 
        #print(threading.active_count())
        self.front_sensor.handle_raw_data() #returns x and y coordinates of only masked points
        
        self.coordinate_transform.x = self.front_sensor.X
        self.coordinate_transform.y = self.front_sensor.Y  
       
        self.coordinate_transform.update_vehicle_states()#gets heading and rotation matrix
        self.count_iter=self.count_iter+1
        
        ## Updating states
        # Update all the related states and variables
        
        # Predict the pos_vector
        self.navigation_controller.predict_pos_vector() #just updates direction of heading in short dr
        # Raw data handling 
        
        # Update x,y used for datapreprocessing using updated front sensor data
        
        #print(np.unique(self.front_sensor.X)

        ## Building necessary map and variabes based on updated states
        # Calculate obstacle vectors 
        self.coordinate_transform.convert_body_to_inertial_frame() #returns 3xn matrix containing n obstacles
        # Create Obstacle map out of obstacle vector
        self.navigation_controller.obstacle_map=None #fetch a new map everytime
        #print(self.coordinate_transform.obstacle_vector_inertial)
        if self.coordinate_transform.obstacle_vector_inertial.size != 0: #inertial map updation logic
            self.navigation_stack()

        #take world map and convert to relative distance map
        if(self.navigation_map.world_map.size!=0):
            self.navigation_controller.obstacle_map = self.navigation_map.convert_inertial_to_rel_world()
            print(self.navigation_controller.obstacle_map)
        

        
        # Minimizing obstacle points for same obstacle by averaging for nearby points       
        #self.map_optimize()
        # self.obstacle_map_optim=self.navigation_map.map
        # self.navigation_controller.obstacle_inertial=self.navigation_map.map
        # self.navigation_controller.obstacle_body=self.navigation_controller.obstacle_map

        self.t11.append(round(time.time()-self.t1_dum,4))
        if(self.count_dum%100==0):
            print('\n',self.t11)
            t11=[]
            #print('\n',self.navigation_map.world_map)
            time.sleep(100)
            #with open('navigation_map.txt', 'a') as f:
               # f.write(str(self.navigation_map.world_map))
            print('\n',self.navigation_map.world_map)

        



       
        

            
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

        #Angular inertial obstacle vector to grid based reading and that is stored in global map
        #self.navigation_map.convert_rel_obstacle_to_inertial(self.navigation_map.grid(self.coordinate_transform.obstacle_vector_inertial.T))
         #updates map in world frame

        self.navigation_map.map=self.navigation_map.grid(self.navigation_map.convert_rel_obstacle_to_inertial1(self.coordinate_transform.obstacle_vector_inertial.T[:,0:2]))#inertial frame grid 
        #self.navigation_map.inertial_grid=np.unique(np.concatenate((self.navigation_map.map, self.navigation_map.inertial_grid),axis=0),axis=0)
        #Relative obstacle is sent to the navigation algorithm
        #**todo send this map to update world map, and extract obstacle map from it
        #add a variable for world_map
        if(self.navigation_map.world_map.size!=0):
            self.navigation_map.world_map=np.unique(np.concatenate((self.navigation_map.world_map,self.navigation_map.map),axis=0),axis=0) ##Todo check uniqueness before adding.
        else:
            self.navigation_map.world_map=self.navigation_map.map
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