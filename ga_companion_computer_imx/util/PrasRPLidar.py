from pyrplidar import PyRPlidar
import time
import math
import threading

class PrasRPLidar_:
    def __init__(self,scan_type,offset,minRange,maxRange): # I had to stop and start the lidar to prevent errors, made it more robust,angle and m, no radians
        self.angles = [] #This stores the raw angles output by the lidar
        self.distances = [] #This stores the raw distances output by the lidar
        self.lidar1 = PyRPlidar()
        self.lidar1.connect(port="/dev/ttyUSB0", baudrate=115200, timeout=3)
        self.lidar1.stop()
        self.lidar1.set_motor_pwm(0)
        self.lidar1.disconnect()
        time.sleep(5)
        self.lidar1.connect(port="/dev/ttyUSB0", baudrate=115200, timeout=3)
        self.lidar1.set_motor_pwm(500)
        time.sleep(2)
        self.maxRange = maxRange
        self.raw_data = [500]*360
        self.offset = offset
        self.minRange = minRange        
        self.scan_type = scan_type
        self.scan_generator = self.lidar1.start_scan_express(self.scan_type)
        self.drivername = 'PrasRPLidar'
        self.time = time.time()
        self.start_time = time.time()
        print('init')
        print(self.raw_data)

    def simple_express_scan(self):
        for count, scan in enumerate(self.scan_generator()):
            q = scan.quality
            a = scan.angle
            d = scan.distance
            s = scan.start_flag
            if(q>170 and d>self.minRange*1000 and d<self.maxRange*1000):
                a = int(math.floor(a))
                if(a+self.offset>359):
                    a = a - 360 + self.offset
                elif(a+self.offset<0):
                    a= a + 360 + self.offset
                else:
                    a=a + self.offset
                if(a==360):
                    a=0
                if(d<self.minRange):
                    self.raw_data[a] = 500
                else:
                    self.raw_data[a] = d/1000
            else:
                a = int(math.floor(a))
                self.raw_data[a] = 500
    def __del__(self):
        self.lidar1.stop()
        self.lidar1.set_motor_pwm(0)
        self.lidar1.disconnect()
        


if __name__ == "__main__":
    obj = PrasRPLidar_(4,0,2,4)#ScanMode,OffsetAngle,minRange and MaxRange
    thr = threading.Thread(target=obj.simple_express_scan)
    thr.start()
    time.sleep(5)
    print("printing in 2 seconds")
    try:
        while True:
            print(time.now())
            print(obj.raw_data[0:30])
            print(time.now())
            time.sleep(1)
    except KeyboardInterrupt:
        print("Closing Program")
        del obj
