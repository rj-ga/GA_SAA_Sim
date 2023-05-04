# GA_IIT_Bombay
Code Structure


Links to follow to set up gazebo and ardupilot:



https://ardupilot.org/dev/docs/building-the-code.html#building-the-code

https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html

https://ardupilot.org/dev/docs/using-gazebo-simulator-with-sitl.html

https://github.com/khancyr/ardupilot_gazebo

https://github.com/ArduPilot/ardupilot



Install Mission Planner:   Version used : 1.3.77 with Ubuntu 20

https://ardupilot.org/planner/docs/mission-planner-installation.html


Commands to start the process:

cd ardupilot/build/sitl/bin && ./arducopter --model  gazebo-iris

cd Listener/build && ./listener

cd algorithm && gazebo --verbose ObstacleWorld.world

mono MissionPlanner-latest/MissionPlanner.exe

Command to run the python script:

python3 pymavlink_main.py --vehicle Test --sitltcp --tcpport 5762

Download this environment: https://www.dropbox.com/s/2vqb7q9gnjf73l3/env.zip?dl=0

Execute:
source env/bin/activate 

This should activate the python3.8 in that shell that should be necessary to run the pymavlink script




