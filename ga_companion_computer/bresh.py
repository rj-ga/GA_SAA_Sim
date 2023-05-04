from math import *
import matplotlib.pyplot as plt
import util.testutil as test
import util.SAAController as control
import numpy as np
import sys
np.set_printoptions(threshold=sys.maxsize)

comp = test.CompanionComputer
con = control.ObstacleAvoidance(max_obs=25)

def grid(point):
    map = np.ones([80,80])*0
    for i in range(len(point)):
        x,y = point[i][0],point[i][1]
        if floor(x) == ceil(x):
            x = x-0.1
        if floor(y) == ceil(y):
            y = y - 0.1

        coordinate_on_map = [(floor(x) + ceil(x))/2,(floor(y) + ceil(y))/2]
        map[ceil(coordinate_on_map[0]),floor(coordinate_on_map[1])] = 1
    return map


#obstacles = [[1.1,2.3],[2.1,2.3],[6.3,0.1],[1,1],[0.6,10]]
obstacles = con.get_obstacle()
print(obstacles)
# i,j= np.where(grid(obstacles)==1)
# points = np.array([[i[k]-0.5,j[k]+0.5] for k in range(len(i))])
# plt.plot(points[:,0],points[:,1],'.')
# plt.plot(np.array(obstacles)[:,0],np.array(obstacles)[:,1],'o')
# plt.grid()
# plt.gca().set_xticks([i for i in range(10)])
# plt.gca().set_yticks([i for i in range(10)])
# plt.show()
