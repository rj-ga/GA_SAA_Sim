import csv
import os
import math
import can
import time
import subprocess
import threading
import numpy as np


class NanoRadar:
    def __init__(self):
        print('init')
        try:
            os.system("modprobe can")
            time.sleep(0.1)
            os.system("modprobe can_raw")
            time.sleep(0.1)
            os.system("ip link set can0 down")
            time.sleep(0.1)
            os.system("ip link set can0 up type can bitrate 500000")
            time.sleep(0.1)

        except:
            print("Exception Received")

        self.__can_bus = can.interface.Bus(channel='can0', bustype='socketcan',
                                            bitrate=500000)

        # can_msg = can.Message(arbitration_id=200,
        #                       is_extended_id=True, dlc=8,
        #                       data=[0, 1, 0, 0, 0, 0, 0, 0],
        #                       channel='can0')
        # self.__can_bus.send(can_msg)

        self.__can_data_received = None
        #self.candump_process = subprocess.Popen(['candump','can0'],stdout=subprocess.PIPE)
        self.b=[]
        self.start_time = time.time()
        self.final_x = []
        self.final_y = []
        self.obj_x_dis = []
        self.obj_y_dis = []
        # t1 = threading.Thread(target=self.empty_readings)
        # t1.start()
        # t2 = threading.Thread(target=self.update)
        # t2.start()
        
    def update(self):
        while True:
            self.__can_data_received = self.__can_bus.recv(0)
            print(self.__can_data_received)
        #     if self.__can_data_received is not None:
        #         self.__can_node_base_id = self.__can_data_received.arbitration_id
        #         print(self.__can_node_base_id)
        #     #print('getting data')
        #     self.output = self.candump_process.stdout.readline()
        #     # self.output = self.__can_bus.recv(0)
        #     #print(self.output)
        #     if not self.output:
        #         break
        #     self.output = self.output.decode().strip()
        #     # #print("Output", self.output)
        #     self.data = self.output.split(' ')
        #     #print("data", self.data)
            
        #     if self.data[2] == '201':
        #         #print("status, ", self.data)
        #         self.b = self.data[7:]
        # #        b = ['30','54','6C','11','80','65','80','A2']update_vars
        #         for i in range(len(self.b)):
        #             self.b[i] = int(self.b[i],16)
        #             #print(self.b[i])


            
        #     if self.data[2] == '700':
        #         print("700", self.data)

            

        #     self.output = self.candump_process.stdout.readline()
        #     if not self.output:
        #         break
        #     self.output = self.output.decode().strip()
        #     # #print("Output", self.output)
        #     self.data = self.output.split(' ')

        #     # if self.data[2] == '60A':
        #     #     print("60A", self.data)

        #     if self.data[2]=='60B':
        #         self.b = self.data[7:]
        # #        b = ['30','54','6C','11','80','65','80','A2']update_vars
        #         for i in range(len(self.b)):
        #             self.b[i] = int(self.b[i],16)
        #         self.obj_id = self.b[0]
        #         self.obj_radial_distance = (self.b[1]*32+(self.b[2]>>3))*0.2 - 500        
        #         self.obj_lateral_range = ((self.b[2]&7)*256 + self.b[3])*0.2-204.6
        #         self.obj_radial_speed = (self.b[4]*4+(self.b[5]>>6))*0.25-128
        #         self.obj_lateral_speed = ((self.b[5]&63)*8+(self.b[6]>>5))*0.25-64
        #         self.obj_rcs = self.b[7]*0.5-64
        #         #print(self.obj_rcs)
        #         self.obj_x_temp_dis = (self.b[1]*32+math.floor(self.b[2])/8)*0.2-500
        #         self.obj_y_temp_dis = ((self.b[2]%8)*256+self.b[3])*0.2-204.6
        #         #set range here
        #         if(self.obj_x_temp_dis*self.obj_x_temp_dis+self.obj_y_temp_dis*self.obj_y_temp_dis<1600):
        #             self.obj_x_dis.append(self.obj_x_temp_dis)
        #             self.obj_y_dis.append(self.obj_y_temp_dis)
        #             #time.sleep(0.003)
        #         #print("radial speed", self.obj_radial_speed)
        #         #print("lateral speed", self.obj_lateral_speed)
        #         #csv_line = str(time.ctime())+','+str(round(obj_x_dis,1))+','+str(round(obj_y_dis,1))+','+str(round(obj_lateral_range,1))+','+str(round(obj_radial_distance,1))
        #         #writer.writerow(csv_line)
        #         #f.write(csv_line+'\n') 
        #         #print((self.obj_x_dis,1),(self.obj_y_dis,1))
            
            
        #     else:
        #         #print()
        #         #print("X,Y,obj_lateral_range")
        #         #print("Exception")
        #         pass
            
        #     #print(self.final_x, self.final_y)
        #     #print(self.obj_x_dis, self.obj_y_dis)

    # def empty_readings(self):
    #     while True:
    #         #print("getting inside")
    #         self.final_x = self.obj_x_dis
    #         self.final_y = self.obj_y_dis
    #         #print(self.final_x, self.final_y)
    #         self.obj_x_dis = []
    #         self.obj_y_dis = []
    #         time.sleep(0.05)


            



    # def initialise_can():
    #     try:
    #         os.system("modprobe can")
    #         time.sleep(0.1)
    #         os.system("modprobe can_raw")
    #         time.sleep(0.1)
    #         os.system("ip link set can0 up type can bitrate 500000")
    #         time.sleep(0.1)

    #     except:
    #         print("Exception Received")

if __name__=="__main__":
     obj = NanoRadar()
     print('init')
    #  t1 = threading.Thread(target=obj.empty_readings)
    #  t1.start()
     t2 = threading.Thread(target=obj.update)
     t2.start()
     print('thread started')


# initialise_can()
# candump_process = subprocess.Popen(['candump','can0'],stdout=subprocess.PIPE)
# b=[]
# start_time = time.time()
# csv_name = str(time.ctime())+'.csv'
# f = open(csv_name,'w')
# #writer = csv.writer(csv_name,delimiter=',')
# while time.time()-start_time<300:
#     output = candump_process.stdout.readline()
#     if not output:
#         break
#     output = output.decode().strip()
#     data = output.split(' ')
    
#     if data[2]=='60B':
#         b = data[7:]
# #        b = ['30','54','6C','11','80','65','80','A2']
#         for i in range(len(b)):
#              b[i] = int(b[i],16)
#         obj_id = b[0]
#         obj_radial_distance = (b[1]*32+(b[2]>>3))*0.2 - 500        
#         obj_lateral_range = ((b[2]&7)*256 + b[3])*0.2-204.6
#         obj_radial_speed = (b[4]*4+(b[5]>>6))*0.25-128
#         obj_lateral_speed = ((b[5]&63)*8+(b[6]>>5))*0.25-64
#         obj_rcs = b[7]*0.5-64
#         obj_y_dis = (b[1]*32+math.floor(b[2])/8)*0.2-500
#         obj_x_dis = ((b[2]%8)*256+b[3])*0.2-204.6
#         csv_line = str(time.ctime())+','+str(round(obj_x_dis,1))+','+str(round(obj_y_dis,1))+','+str(round(obj_lateral_range,1))+','+str(round(obj_radial_distance,1))
#         #writer.writerow(csv_line)
#         f.write(csv_line+'\n') 
#         print(round(obj_x_dis,1),round(obj_y_dis,1))
	
#     else:
#         print()
#         print("X,Y,obj_lateral_range")
	
        
# f.close()
	
