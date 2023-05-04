import serial
import time
import numpy as np
from csv_logger import CsvLogger
import logging
import threading
from threading import Thread
from datetime import datetime
#import pyqtgraph as pg
#from pyqtgraph.Qt import QtWidgets

# Change the configuration file name
#configFileName = 'AWR1843config.cfg'
filename = './ti_log'+str(datetime.now())+'.csv'
delimiter = ','
level = logging.INFO
custom_additional_levels = ['logs_a', 'logs_b']
fmt = f'%(asctime)s{delimiter}%(levelname)s{delimiter}%(message)s%(created)f'
datefmt = '%Y/%m/%d %H:%M:%S'
#max_size = 1024*(10^6)  # 1 kilobyte
#max_files = 10 # 4 rotating files
header = ['date', 'level', 'x', 'y', 'z']

csvLogger = CsvLogger(filename=filename,
                       delimiter=delimiter,
                       level=level,
                       add_level_names=custom_additional_levels,
                       add_level_nums=None,
                       fmt=fmt,
                       datefmt=datefmt,
 #                      max_size=max_size,
 #                      max_files=max_files,
                       header=header)



class TiRadar():
    def __init__(self, configFileName):
        #Thread.__init__(self)
        self.daemon = True
        #self.start()
        self.configFileName = configFileName
        self.CLIport = {}
        self.self.Dataport = {}
        self.byteBuffer = np.zeros(2**15,dtype = 'uint8')
        self.byteBufferLength = 0
        self.detObj = {}
        self.x = []
        self.y = []
        self.z = []


    def serialConfig(self, configFileName):
        
        global CLIport
        global self.Dataport
        # Open the serial ports for the configuration and the data ports
        
        # Raspberry pi
        CLIport = serial.Serial('/dev/ttyACM0', 115200)
        self.Dataport = serial.Serial('/dev/ttyACM1', 921600)
        
        # Windows
        # CLIport = serial.Serial('/dev/ttyUSB1', 115200)
        # self.Dataport = serial.Serial('/dev/ttyUSB2', 921600)

        # Read the configuration file and send it to the boardIndexError: index 4 is out of bounds for axis 0 with size 4
        config = [line.rstrip('\r\n') for line in open(self.configFileName)]
        for i in config:
            CLIport.write((i+'\n').encode())
            print(i)
            time.sleep(0.01)
            
        return CLIport, self.Dataport

    # Function to parse the data inside the configuration file 
    def parseConfigFile(self, configFileName):
        self.configParameters = {} # Initialize an empty dictionary to store the configuration parameters
        
        # Read the configuration file and send it to the board
        config = [line.rstrip('\r\n') for line in open(self.configFileName)]
        for i in config:
            
            # Split the line
            splitWords = i.split(" ")
            
            # Hard code the number of antennas, change if other configuration is used
            numRxAnt = 4
            numTxAnt = 3
            
            # Get the information about the profile configuration
            if "profileCfg" in splitWords[0]:
                startFreq = int(float(splitWords[2]))
                idleTime = int(splitWords[3])
                rampEndTime = float(splitWords[5])
                freqSlopeConst = float(splitWords[8])
                numAdcSamples = int(splitWords[10])
                numAdcSamplesRoundTo2 = 1;
                
                while numAdcSamples > numAdcSamplesRoundTo2:
                    numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2;
                    
                digOutSampleRate = int(splitWords[11]);
                
            # Get the information about the frame configuration    
            elif "frameCfg" in splitWords[0]:
                
                chirpStartIdx = int(splitWords[1]);
                chirpEndIdx = int(splitWords[2]);
                numLoops = int(splitWords[3]);
                numFrames = int(splitWords[4]);
                framePeriodicity = float(splitWords[5]);

                
        # Combine the read data to obtain the configuration parameters           
        numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
        self.configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
        self.configParameters["numRangeBins"] = numAdcSamplesRoundTo2
        self.configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * numAdcSamples)
        self.configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * self.configParameters["numRangeBins"])
        self.configParameters["dopplerResolutionMps"] = 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * self.configParameters["numDopplerBins"] * numTxAnt)
        self.configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate)/(2 * freqSlopeConst * 1e3)
        self.configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)
        
        return self.configParameters

# ------------------------------------------------------------------

    # Funtion to read and parse the incoming data
    def readAndParseData18xx(self, self.Dataport, configParameters):
        global byteBuffer
        global byteBufferLength
        
        # Constants
        OBJ_STRUCT_SIZE_BYTES = 12;
        BYTE_VEC_ACC_MAX_SIZE = 2**15;
        MMWDEMO_UART_MSG_DETECTED_POINTS = 1;
        MMWDEMO_UART_MSG_RANGE_PROFILE   = 2;
        maxBufferSize = 2**15;
        tlvHeaderLengthInBytes = 8;
        pointLengthInBytes = 16;
        magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
        
        # Initialize variables
        self.magicOK = 0 # Checks if magic number has been read
        self.dataOK = 0 # Checks if the data has been read correctly
        self.frameNumber = 0
        self.detObj = {}
        
        readBuffer = self.Dataport.read(self.Dataport.in_waiting)
        byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
        byteCount = len(byteVec)
        
        # Check that the buffer is not full, and then add the data to the buffer
        if (self.byteBufferLength + byteCount) < maxBufferSize:
            self.byteBuffer[self.byteBufferLength:self.byteBufferLength + byteCount] = byteVec[:byteCount]
            self.byteBufferLength = self.byteBufferLength + byteCount
            
        # Check that the buffer has some data
        if self.byteBufferLength > 16:
            
            # Check for all possible locations of the magic word
            possibleLocs = np.where(self.byteBuffer == magicWord[0])[0]

            # Confirm that is the beginning of the magic word and store the index in startIdx
            startIdx = []
            for loc in possibleLocs:
                check = self.byteBuffer[loc:loc+8]
                if np.all(check == magicWord):
                    startIdx.append(loc)
                
            # Check that startIdx is not empty
            if startIdx:
                
                # Remove the data before the first start index
                if startIdx[0] > 0 and startIdx[0] < self.byteBufferLength:
                    self.byteBuffer[:self.byteBufferLength-startIdx[0]] = self.byteBuffer[startIdx[0]:self.byteBufferLength]
                    self.byteBuffer[self.byteBufferLength-startIdx[0]:] = np.zeros(len(self.byteBuffer[self.byteBufferLength-startIdx[0]:]),dtype = 'uint8')
                    self.byteBufferLength = self.byteBufferLength - startIdx[0]
                    
                # Check that there have no errors with the byte buffer length
                if self.byteBufferLength < 0:
                    self.byteBufferLength = 0
                    
                # word array to convert 4 bytes to a 32 bit number
                word = [1, 2**8, 2**16, 2**24]
                
                # Read the total packet length
                totalPacketLen = np.matmul(self.byteBuffer[12:12+4],word)
                
                # Check that all the packet has beenIndexError: index 4 is out of bounds for axis 0 with size 4 read
                if (self.byteBufferLength >= totalPacketLen) and (self.byteBufferLength != 0):
                    self.magicOK = 1
        
        # If magicOK is equal to 1 then process the message
        if self.magicOK:
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]
            
            # Initialize the pointer index
            idX = 0
            
            # Read the header
            magicNumber = self.byteBuffer[idX:idX+8]
            idX += 8
            version = format(np.matmul(self.byteBuffer[idX:idX+4],word),'x')
            idX += 4
            totalPacketLen = np.matmul(self.byteBuffer[idX:idX+4],word)
            idX += 4
            platform = format(np.matmul(self.byteBuffer[idX:idX+4],word),'x')
            idX += 4
            frameNumber = np.matmul(self.byteBuffer[idX:idX+4],word)
            idX += 4
            timeCpuCycles = np.matmul(self.byteBuffer[idX:idX+4],word)
            idX += 4
            numDetectedObj = np.matmul(self.byteBuffer[idX:idX+4],word)
            idX += 4
            numTLVs = np.matmul(self.byteBuffer[idX:idX+4],word)
            idX += 4
            subFrameNumber = np.matmul(self.byteBuffer[idX:idX+4],word)
            idX += 4

            # Read the TLV messages
            for tlvIdx in range(numTLVs):
                
                # word array to convert 4 bytes to a 32 bit number
                word = [1, 2**8, 2**16, 2**24]

                # Check the header of the TLV message
                tlv_type = np.matmul(self.byteBuffer[idX:idX+4],word)
                idX += 4
                tlv_length = np.matmul(self.byteBuffer[idX:idX+4],word)
                idX += 4

                # Read the data depending on the TLV message
                if tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS:

                    # Initialize the arrays
                    x = np.zeros(numDetectedObj,dtype=np.float32)
                    y = np.zeros(numDetectedObj,dtype=np.float32)
                    z = np.zeros(numDetectedObj,dtype=np.float32)
                    velocity = np.zeros(numDetectedObj,dtype=np.float32)
                    
                    for objectNum in range(numDetectedObj):
                        
                        # Read the data for each object
                        x[objectNum] = self.byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        y[objectNum] = self.byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        z[objectNum] = self.byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        velocity[objectNum] = self.byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                    
                    # Store the data in the detObj dictionary
                    self.detObj = {"numObj": numDetectedObj, "x": x, "y": y, "z": z, "velocity":velocity}
                    self.dataOK = 1
                    
    
            # Remove already processed data
            if idX > 0 and self.byteBufferLength>idX:
                shiftSize = totalPacketLen
                
                    
                self.byteBuffer[:self.byteBufferLength - shiftSize] = self.byteBuffer[shiftSize:self.byteBufferLength]
                self.byteBuffer[self.byteBufferLength - shiftSize:] = np.zeros(len(self.byteBuffer[self.byteBufferLength - shiftSize:]),dtype = 'uint8')
                self.byteBufferLength = self.byteBufferLength - shiftSize
                
                # Check that there are no errors with the buffer length
                if self.byteBufferLength < 0:
                    self.byteBufferLength = 0         

        return self.dataOK, self.frameNumber, self.detObj
    
    # Function to update the data and display in the plot
    def update(self):
        self.dataOk = 0
        self.detObj
        x = []
        y = []
        # Read and parse the received data
        while True:
            self.dataOk, self.frameNumber, self.detObj = self.readAndParseData18xx(self.Dataport, self.configParameters)
            if self.dataOk and len(self.detObj["x"])>0:
                self.x = -self.detObj["x"]
                self.y = self.detObj["y"]
                self.z = self.detObj["z"]
                #s.setData(x,y)
                #QtWidgets.QApplication.processEvents()
                #print(self.detObj)
                #print(x)
                #print(y)
                print("inside uart", self.z)
                #time.sleep(0.1)
                if self.dataOk:
                    for i in range(0, len(self.detObj['x'])):
                        csvLogger.logs_a([x[i], y[i], z[i]])
        return self.dataOk


# if __name__ == "__main__":

#     obj = TiRadar('config.cfg')
#     CLIport, self.Dataport = obj.serialConfig(obj.configFileName)

#     # Get the configuration parameters from the configuration file
#     configParameters = obj.parseConfigFile(obj.configFileName)

    # START QtAPPfor the plot
    #app = QtWidgets.QApplication([])

    # Set the plot 
    # pg.setConfigOption('background','w')
    # win = pg.GraphicsLayoutWidget(title="2D scatter plot")
    # p = win.addPlot()
    # p.setXRange(-5,5)
    # p.setYRange(0,15)
    # p.setLabel('left',text = 'Y position (m)')
    # p.setLabel('bottom', text= 'X position (m)')
    # s = p.plot([],[],pen=None,symbol='o')
    # win.show()

    # all_logs = csvLogger.get_logs(evaluate=False)
    # for log in all_logs:
    #     print(log)


    #Main loop 
    # detObj = {}  
    # frameData = {}    
    # currentIndex = 0
    # t = threading.Thread(target=obj.update)
    # t.start()
    # while True:
    #     try:
    #         # Update the data and check if the data is okay
    #         dataOk = obj.update()
    #         #t.start()
    #         if dataOk:
    #             # Store the current frame into frameData
    #             frameData[currentIndex] = obj.detObj
    #             currentIndex += 1
            
    #         time.sleep(0.05) # Sampling frequency of 30 Hz
            
    #     # Stop the program and close everything if Ctrl + c is pressed
    #     except KeyboardInterrupt:
    #         CLIport.write(('sensorStop\n').encode())
    #         CLIport.close()
    #         self.Dataport.close()
    #         break
# # Function to parse the data inside the configuration file
# # def parseConfigFile(configFileName):
# #     configParameters = {} # Initialize an empty dictionary to store the configuration parameters
    
# #     # Read the configuration file and send it to the board
# #     config = [line.rstrip('\r\n') for line in open(configFileName)]
# #     for i in config:
        
# #         # Split the line
# #         splitWords = i.split(" ")
        
# #         # Hard code the number of antennas, change if other configuration is used
# #         numRxAnt = 4
# #         numTxAnt = 3
        
# #         # Get the information about the profile configuration
# #         if "profileCfg" in splitWords[0]:
# #             startFreq = int(float(splitWords[2]))
# #             idleTime = int(splitWords[3])
# #             rampEndTime = float(splitWords[5])
# #             freqSlopeConst = float(splitWords[8])
# #             numAdcSamples = int(splitWords[10])
# #             numAdcSamplesRoundTo2 = 1;
            
# #             while numAdcSamples > numAdcSamplesRoundTo2:
# #                 numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2;
                
# #             digOutSampleRate = int(splitWords[11]);
            
# #         # Get the information about the frame configuration    
# #         elif "frameCfg" in splitWords[0]:
            
# #             chirpStartIdx = int(splitWords[1]);
# #             chirpEndIdx = int(splitWords[2]);
# #             numLoops = int(splitWords[3]);
# #             numFrames = int(splitWords[4]);
# #             framePeriodicity = float(splitWords[5]);

            
# #     # Combine the read data to obtain the configuration parameters           
# #     numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
# #     configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
# #     configParameters["numRangeBins"] = numAdcSamplesRoundTo2
# #     configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * numAdcSamples)
# #     configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
# #     configParameters["dopplerResolutionMps"] = 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
# #     configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate)/(2 * freqSlopeConst * 1e3)
# #     configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)
    
# #     return configParameters
   
# # ------------------------------------------------------------------

# # # Funtion to read and parse the incoming data
# # def readAndParseData18xx(self.Dataport, configParameters):
# #     global byteBuffer, self.byteBufferLength
    
# #     # Constants
# #     OBJ_STRUCT_SIZE_BYTES = 12;
# #     BYTE_VEC_ACC_MAX_SIZE = 2**15;
# #     MMWDEMO_UART_MSG_DETECTED_POINTS = 1;
# #     MMWDEMO_UART_MSG_RANGE_PROFILE   = 2;
# #     maxBufferSize = 2**15;
# #     tlvHeaderLengthInBytes = 8;
# #     pointLengthInBytes = 16;
# #     magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
    
# #     # Initialize variables
# #     magicOK = 0 # Checks if magic number has been read
# #     dataOK = 0 # Checks if the data has been read correctly
# #     frameNumber = 0
# #     detObj = {}
    
# #     readBuffer = self.Dataport.read(self.Dataport.in_waiting)
# #     byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
# #     byteCount = len(byteVec)
    
# #     # Check that the buffer is not full, and then add the data to the buffer
# #     if (self.byteBufferLength + byteCount) < maxBufferSize:
# #         byteBuffer[self.byteBufferLength:self.byteBufferLength + byteCount] = byteVec[:byteCount]
# #         self.byteBufferLength = self.byteBufferLength + byteCount
        
# #     # Check that the buffer has some data
# #     if self.byteBufferLength > 16:
        
# #         # Check for all possible locations of the magic word
# #         possibleLocs = np.where(byteBuffer == magicWord[0])[0]

# #         # Confirm that is the beginning of the magic word and store the index in startIdx
# #         startIdx = []
# #         for loc in possibleLocs:
# #             check = byteBuffer[loc:loc+8]
# #             if np.all(check == magicWord):
# #                 startIdx.append(loc)
               
# #         # Check that startIdx is not empty
# #         if startIdx:
            
# #             # Remove the data before the first start index
# #             if startIdx[0] > 0 and startIdx[0] < self.byteBufferLength:
# #                 byteBuffer[:self.byteBufferLength-startIdx[0]] = byteBuffer[startIdx[0]:self.byteBufferLength]
# #                 byteBuffer[self.byteBufferLength-startIdx[0]:] = np.zeros(len(byteBuffer[self.byteBufferLength-startIdx[0]:]),dtype = 'uint8')
# #                 self.byteBufferLength = self.byteBufferLength - startIdx[0]
                
# #             # Check that there have no errors with the byte buffer length
# #             if self.byteBufferLength < 0:
# #                 self.byteBufferLength = 0
                
# #             # word array to convert 4 bywhile True:
# #     try:
# #         # Update the data and check if the data is okay
# #         dataOk = obj.update()
# #         #t.start()
# #         if dataOk:
# #             # Store the current frame into frameData
# #             frameData[currentIndex] = obj.detObj
# #             currentIndex += 1
        
# #         time.sleep(0.05) # Sampling frequency of 30 Hz

# #         # Stop the program and close everything if Ctrl + c is pressed
# #     except KeyboardInterrupt:
# #         CLIport.write(('sensorStop\n').encode())
# #         CLIport.close()
# #         self.Dataport.close()
# #         breaktes to a 32 bit number
# #             word = [1, 2**8, 2**16, 2**24]
            
# #             # Read the total packet length
# #             totalPacketLen = np.matmul(byteBuffer[12:12+4],word)
            
# #             # Check that all the packet has beenIndexError: index 4 is out of bounds for axis 0 with size 4 read
# #             if (self.byteBufferLength >= totalPacketLen) and (self.byteBufferLength != 0):
# #                 magicOK = 1
    
# #     # If magicOK is equal to 1 then process the message
# #     if magicOK:
# #         # word array to convert 4 bytes to a 32 bit number
# #         word = [1, 2**8, 2**16, 2**24]
        
# #         # Initialize the pointer index
# #         idX = 0
        
# #         # Read the header
# #         magicNumber = byteBuffer[idX:idX+8]
# #         idX += 8
# #         version = format(np.matmul(byteBuffer[idX:idX+4],word),'x')
# #         idX += 4
# #         totalPacketLen = np.matmul(byteBuffer[idX:idX+4],word)
# #         idX += 4
# #         platform = format(np.matmul(byteBuffer[idX:idX+4],word),'x')
# #         idX += 4
# #         frameNumber = np.matmul(byteBuffer[idX:idX+4],word)
# #         idX += 4
# #         timeCpuCycles = np.matmul(byteBuffer[idX:idX+4],word)
# #         idX += 4
# #         numDetectedObj = np.matmul(byteBuffer[idX:idX+4],word)
# #         idX += 4
# #         numTLVs = np.matmul(byteBuffer[idX:idX+4],word)
# #         idX += 4
# #         subFrameNumber = np.matmul(byteBuffer[idX:idX+4],word)
# #         idX += 4

# #         # Read the TLV messages
# #         for tlvIdx in range(numTLVs):
            
# #             # word array to convert 4 bytes to a 32 bit number
# #             word = [1, 2**8, 2**16, 2**24]

# #             # Check the header of the TLV message
# #             tlv_type = np.matmul(byteBuffer[idX:idX+4],word)
# #             idX += 4
# #             tlv_length = np.matmul(byteBuffer[idX:idX+4],word)
# #             idX += 4

# #             # Read the data depending on the TLV message
# #             if tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS:

# #                 # Initialize the arrays
# #                 x = np.zeros(numDetectedObj,dtype=np.float32)
# #                 y = np.zeros(numDetectedObj,dtype=np.float32)
# #                 z = np.zeros(numDetectedObj,dtype=np.float32)
# #                 velocity = np.zeros(numDetectedObj,dtype=np.float32)
                
# #                 for objectNum in range(numDetectedObj):
                    
# #                     # Read the data for each object
# #                     x[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
# #                     idX += 4
# #                     y[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
# #                     idX += 4
# #                     z[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
# #                     idX += 4
# #                     velocity[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
# #                     idX += 4
                
# #                 # Store the data in the detObj dictionary
# #                 detObj = {"numObj": numDetectedObj, "x": x, "y": y, "z": z, "velocity":velocity}
# #                 dataOK = 1
                
 
# #         # Remove already processed data
# #         if idX > 0 and self.byteBufferLength>idX:
# #             shiftSize = totalPacketLen
            
                
# #             byteBuffer[:self.byteBufferLength - shiftSize] = byteBuffer[shiftSize:self.byteBufferLength]
# #             byteBuffer[self.byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[self.byteBufferLength - shiftSize:]),dtype = 'uint8')
# #             self.byteBufferLength = self.byteBufferLength - shiftSize
            
# #             # Check that there are no errors with the buffer length
# #             if self.byteBufferLength < 0:
# #                 self.byteBufferLength = 0         

# #     return dataOK, frameNumber, detObj

# # ------------------------------------------------------------------

# # # Function to update the data and display in the plot
# # def update():
     
# #     dataOk = 0
# #     global detObj
# #     x = []
# #     y = []
      
# #     # Read and parse the received data
# #     dataOk, frameNumber, detObj = readAndParseData18xx(self.Dataport, configParameters)
    
# #     if dataOk and len(detObj["x"])>0:
# #         x = -detObj["x"]
# #         y = detObj["y"]
# #         z = detObj["z"]
# #         s.setData(x,y)
# #         QtWidgets.QApplication.processEvents()
# #         #print(detObj)
# #         #print(x)
# #         #print(y)
# #         print(z)
# #         if dataOk:
# #             for i in range(0, len(detObj['x'])):
# #                 csvLogger.logs_a([x[i], y[i], z[i]])
# #     return dataOk


# # -------------------------    MAIN   -----------------------------------------  

# # Configurate the serial port
# # CLIport, self.Dataport = serialConfig(configFileName)

# # # Get the configuration parameters from the configuration file
# # configParameters = parseConfigFile(configFileName)

# # # START QtAPPfor the plot
# # app = QtWidgets.QApplication([])

# # # Set the plot 
# # pg.setConfigOption('background','w')
# # win = pg.GraphicsLayoutWidget(title="2D scatter plot")
# # p = win.addPlot()
# # p.setXRange(-5,5)
# # p.setYRange(0,15)
# # p.setLabel('left',text = 'Y position (m)')
# # p.setLabel('bottom', text= 'X position (m)')
# # s = p.plot([],[],pen=None,symbol='o')
# # win.show()

# # all_logs = csvLogger.get_logs(evaluate=False)
# # for log in all_logs:
# #     print(log)


# # # Main loop 
# # detObj = {}  
# # frameData = {}    
# # currentIndex = 0
# # while True:
# #     try:
# #         # Update the data and check if the data is okay
# #         dataOk = update()
        
# #         if dataOk:
# #             # Store the current frame into frameData
# #             frameData[currentIndex] = detObj
# #             currentIndex += 1
        
# #         time.sleep(0.05) # Sampling frequency of 30 Hz
        
# #     # Stop the program and close everything if Ctrl + c is pressed
# #     except KeyboardInterrupt:
# #         CLIport.write(('sensorStop\n').encode())
# #         CLIport.close()
# #         self.Dataport.close()
# #         win.close()
# #         break
        
    





