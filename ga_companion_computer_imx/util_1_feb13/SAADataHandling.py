"""
@author: Dhruv Parikh
@organisation: General Aeronautics Pvt. Ltd.
"""

"""
@Description:
    Data handling for the obstacle avoidance sensor data. 

    @Todo: Handle multiple sensor 
    Pack all the data in one stream
    Handle the estimation errors and filtering portion 
    Account for offsets (Least priority)
"""

import math
import numpy as np
import util.VectorMath as vmath
import logging
import time
import sys
np.set_printoptions(threshold=sys.maxsize)

class Sensor():
    #This class handles single instance of reading and has no memory

    def __init__(self,id,delta_angle,max_range,min_range,init_angle):
        """
        @Params: id: sensor id for future
        delta_angle: for the steps in lidar data (dtheta)
        max_range: maximum range of sensor
        min_range: minimum range of sensor
        init_angle: Angle from which the sensor will start recording (offset)
        """

        #Assign to class params
        self.id = id
        self.delta_angle = delta_angle
        self.max_range = max_range
        self.min_range = min_range
        self.init_angle = init_angle

        self.init_index = round(init_angle/delta_angle)

        self.master_array_length = round(2*math.pi/delta_angle)


        # self.X = np.array([])
        # self.Y = np.array([])

        self.X = [0]*360
        self.Y = [0]*360

        self.data = [0]
        self.count = 0



    def handle_raw_data(self):
        #5.888938903808594e-05 seconds
        data = self.data
        self.X=[]
        self.Y=[]
        
        #self.X = np.array([])
        #self.Y = np.array([]) 
        # self.X = [0]*len(data)
        # self.Y = [0]*len(data)
        #If data is too less
        if data != None:
            if len(data)<=10:
                logging.info("Data not populated in other thread!!")

            else:

                #Create a bit mask for taking trusty readings of 90% 
                #@TODO: This should be based on CC code params because we can't afford to loose data in low range sensors
                sensor_binary_mask = [1.1*self.min_range<=abs(i)<=0.95*self.max_range for i in data]

                #Recreating the X and Y vectors 
                
                #If any reading falls in range
                #if any(sensor_binary_mask) == True:
                for i in range(len(data)):
                    #Populate only those readings where the range is true else don't bother calculating
                    #if sensor_binary_mask[i] == True:
                        #Piecing out magnitude to X-Y coordinates
                        self.count+=1
                        if(data[i]<100):
                            self.X.append(float(data[i]*math.sin(self.delta_angle*i + self.init_angle)))
                            self.Y.append(float(data[i]*math.cos(self.delta_angle*i + self.init_angle)))
                        # self.X[i] = float(data[i]*math.sin(self.delta_angle*i + self.init_angle))
                        # self.Y[i] = float(data[i]*math.cos(self.delta_angle*i + self.init_angle))
                        # '
                        # if(self.X.size==0):
                        #     self.X=np.array([float(data[i]*math.sin(self.delta_angle*i + self.init_angle))])
                        #     self.Y=np.array([float(data[i]*math.cos(self.delta_angle*i + self.init_angle))])
                            
                            
                        # else:
                        #     self.X=np.concatenate((self.X,np.array([float(data[i]*math.sin(self.delta_angle*i + self.init_angle))])),axis=0)
                        #     self.Y=np.concatenate((self.Y,np.array([float(data[i]*math.cos(self.delta_angle*i + self.init_angle))])),axis=0)
                                

                            # self.X[i + self.init_index] = float(data[i]*math.cos(self.delta_angle*i + self.init_angle))
                            # self.Y[i + self.init_index] = float(data[i]*math.sin(self.delta_angle*i + self.init_angle))
            

    def combine_multiple_readings(self,x2,y2):
        for i in range(self.master_array_length):
            if x2[i] <40 or y2[i] <40:
                self.X[i] = x2[i]
                self.Y[i] = y2[i]

        return self.X, self.Y

class DataPreProcessor():
    """
    Handle obstacle vector - 1 shot scan
    """
    def __init__(self):
        self.transformations = vmath.AngularTransformation()

        #x and y are the mags
        self.x = np.array([])
        self.y = np.array([])
        self.errorC = 0
        self.Nerror = 0

        #Vehicle states
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.px = 0
        self.py = 0
        self.pz = 0
        
        #Initialise the body to inertial matrix as 3x3 identity
        self.b2i_matrix = np.eye(3)
        self.obstacle_vector_inertial = np.zeros([2,2])

        # Calculate drone's heading vector (x,y,z)
        self.heading = np.zeros(3)


    def update_vehicle_states(self):
        #0.00023102760314941406 seconds

        #Calculate trigs and matrices before hand to reduce computational load
        self.transformations.calc_trig_values(self.roll,self.pitch,self.yaw)
        self.b2i_matrix = (self.transformations.euler_zyx())
        #changed this for field experiments
        
        #self.b2i_matrix = (self.transformations.euler_zyx())
        # Update heading of drone
        self.heading = np.dot(self.b2i_matrix, np.array([1,0,0]))



    def convert_body_to_inertial_frame(self):
        #5.7697296142578125e-05 seconds
        height_allowance = -10

        #These x and y are updated in the mainloop
        x = self.x
        y = self.y
        z = np.zeros([len(x)])
        #print(self.pz)

        #Don't store in map, if the obstacles are below a certain height. 
        #if(self.pz)>=height_allowance:

        #print('entering loop')
        #Obstacle vector in the body frame, unique removed since already data given by lidar shall be unique
        obstacle_vector_body = np.array([x,y,z])#3*n,axis 1 means uniqueness along columns
        #Convert from body to inertial frame if and only if x and y were synced. 
        if(len(x)==len(y)):
            self.obstacle_vector_inertial = np.dot(self.b2i_matrix,obstacle_vector_body)
            #3xn,check for uniqueness of columns
            #print(self.obstacle_vector_inertial)
            
        
class DataPostProcessor():
    """
    Handle the entire obstacle information
    """
    def __init__(self):
        self.map_x = [0]
        self.map_y = [0]
        self.px = 0
        self.py = 0
        self.map = np.array([[]])
        self.math = vmath.vector()
        self.world_map=np.array([[]])


    def cleanup(self):
        k = 0
        dist = [(self.px-b[i][0])**2 + (self.py-b[i][1])**2 for i in range(np.size(b,axis=0))]
        for i in range(len(dist)):
            j = dist[i]
            if j>1600:
                b = np.delete(b, i-k,0)
                k = k+1
        return b
    
    def grid(self,point):
        """
        Inefficient grid maker

        Make a map of 40 x 40 meters around drone. Stores each point as a 1x1 square with center point known. 
        I will only return the obstacles with the center point. 

        The grid points can be stored in map later. 
        """

        #make a map of 40x40    
        map1 = np.array([[]])
        #@TODO: Remove the for loop

        for i in range(len(point)):

            x,y = point[i,0],point[i,1]
            #Get the signs
            sign_x = self.math.return_sign(x)
            sign_y = self.math.return_sign(y)
            #Remove the signs
            x = abs(x)
            y = abs(y)
            #Edge Case handling
            if math.floor(x) == math.ceil(x):
                x = x - 0.1
            if math.floor(y) == math.ceil(y):
                y = y - 0.1

            #Center of the pixel/grid
            coordinate_on_map = np.array([[sign_x*(math.floor(x) + math.ceil(x))/2,sign_y*(math.floor(y) + math.ceil(y))/2]])
            #Add to the map
            if(i==0):
                map1=coordinate_on_map
            elif(coordinate_on_map not in map1):
                map1 = np.concatenate((map1,coordinate_on_map),axis=0)
            else:
                pass
            #map[sign_x*math.ceil(coordinate_on_map[0]),sign_y*math.floor(coordinate_on_map[1])] = 1
        #Return only unique values
        return map1

    def convert_rel_obstacle_to_inertial(self,points):
        """
        Converts the vector from relative frame to origin frame
        """
        
        #Converted to inertial frame
        points = points + np.array([[round(self.px,2),round(self.py,2)]])
        self.map_maker(points) #commented out since once I have rejected all entries, I will make sense of whatever I get

    def convert_rel_obstacle_to_inertial1(self,points):
        """
        Converts the vector from relative frame to origin frame
        """
        
        #Converted to inertial frame
        points = points + np.array([[round(self.px,2),round(self.py,2)]])
        return points#commented out since once I have rejected all entries, I will make sense of whatever I get

        

    def map_maker(self,points):
        """
        You might be wondering why this is a seperate function. But in future there will be a need for a better algorithm such as vortex grid or occupancy grid
        """
        #self.map = np.array([[40,40]])
        self.map = np.unique(np.concatenate((self.map,points),axis=0),axis=0) #makes a list of points
        self.clean_near_obstacles()
        self.forget_far_obstacles()

    def convert_inertial_to_rel(self):
        """Convert the vector from inertial frame to relative frame
        """
        return self.map - np.array([[self.px,self.py]])

    def convert_inertial_to_rel_world(self):
        """Convert the vector from inertial frame to relative frame
        """
        return self.world_map - np.array([[self.px,self.py]])

    def convert_inertial_to_rel_avoid(self,points):   # This function used by navigation_stack
        """Convert the vector from inertial frame to relative frame
        """
        return points - np.array([self.px,self.py])

    def convert_rel_to_inertial_avoid(self,points):   # This function used by navigation_stack
        """Convert the vector from relative frame to inertial frame
        """
        return points + np.array([[self.px,self.py]])

    def give_position(self):
        return np.array([self.px, self.py])

    def clean_near_obstacles(self):
        """
        Removes obstacle within 1 meter radius
        -> Calculated risks are taken here... if drone hasn't stopped for obstacles before 1 meter, then it will probably be noise?
        """
        rel_distance_map = self.convert_inertial_to_rel()
        indices_to_delete = []
        for i in range(np.size(rel_distance_map,axis=0)):
            if self.math.mag2d(rel_distance_map[i,:])<=1:
                indices_to_delete.append(i)
            
        if len(indices_to_delete)>0:
            self.map = np.delete(self.map, indices_to_delete, axis=0)

    def forget_far_obstacles(self):
        """
        Delete obstacles that are very far from the drone 
        We are very short on memory, this keeps the map size within acceptable limits
        """
        rel_distance_map = self.convert_inertial_to_rel()
        indices_to_delete = []
        for i in range(np.size(rel_distance_map,axis=0)):
            if self.math.mag2d(rel_distance_map[i,:])>=30:
                indices_to_delete.append(i)
            
        if len(indices_to_delete)>0:
            self.map = np.delete(self.map, indices_to_delete, axis=0)
            