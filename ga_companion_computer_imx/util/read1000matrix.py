import time
import random
n = int(input("Enter the dimension of the matrix "))
obstacle_num = int(input("ENter the number of obstacles to be added randomly "))
a = [[0]*n for _ in range(n)] #To represent matrix

print(a)

t = time.time()
for i in range(0,obstacle_num,1):
    b = random.sample(range(0,n-1),2)
    print(b[0])
    a[b[0]][b[1]]=1

def find_near_space():
    x = currentPoint[0]
    y=currentPoint[1]
    if(a[x][y+2]==0 and a[x][y+1]==0):
        return(1)
    if(a[x+2][y+2]==0 and [currentPoint[0]+1][currentPoint[1]+1]==0):
        return(2)
    if(a[currentPoint[0]-2][currentPoint[1]+2]==0 and [currentPoint[0]-1][currentPoint[1]+1]==0):
        return(3)
    
    
    
        




def Printing_Map():
    for i in range(n-1,-1,-1):
        for j in range(n):
            print(a[i][j]," ",end="")
        print()

Printing_Map()

sx,sy = int(input("Enter Start Coordinate X: ")),int(input("Enter Start Coordinate Y: "))
ex,ey = int(input("Enter Stop Coordinates X: ")),int(input("Enter Stop Coordinates Y: "))

a[sx][sy] = '@'
a[ex][ey] = '#'
p = '$'
currentPoint = [sx,sy]




Printing_Map()
print(find_near_space())
