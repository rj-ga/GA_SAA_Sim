import threading
import time
from Ti_UART import TiRadar

obj = TiRadar('config.cfg')
CLIport, self.Dataport = obj.serialConfig(obj.configFileName)

# Get the configuration parameters from the configuration file
configParameters = obj.parseConfigFile(obj.configFileName)

# Main loop 
detObj = {}  
frameData = {}    
currentIndex = 0


t = threading.Thread(target=obj.update)
t.start()