import numpy as np
import matplotlib.pyplot as plt
from OptexCD22 import sensorCtrl
from EnderPrinter import printerCtrl
import time


mySensor = sensorCtrl(port='COM5', baudrate=9600)
mySensor.connect_rs485()
xhome = 70.0
yhome = 115.0
zhome = 10.0

myPrinter = printerCtrl(port='COM6',printer='generic', home=[xhome, yhome, zhome])
myPrinter.Start_Control()
xCent = xhome
yCent = yhome

# if mySensor.check_connected():
myPrinter.Go_Center([xCent, yCent], delay=5)
# print('here')
n = 100
x = np.linspace(0,1, n)
vals = np.full(n, np.nan)
r = 45 # radius of circle
theta = np.linspace(0, np.pi*2, n)

print('Recording Data')
'''Circle'''
#point=[xCent+r*np.cos(0), yCent+r*np.sin(0), 15]
'''Grid'''
b = 80
xVals = np.full(n**2, np.nan)
yVals = np.full(n**2, np.nan)
zVals = np.full(n**2, np.nan)
point = [xCent-b, yCent-b, 20]
myPrinter.Go_Point(point, delay=2)
for i in range(n):
    # print('New Point')
    '''Circle'''
    #point=[xCent+r*np.cos(theta[i]), yCent+r*np.sin(theta[i]), 15]
    
    '''Grid'''
    for j in range(n):
        xVals[(i*n)+j] = xCent-b + 2*b/n*j
        yVals[(i*n)+j] = yCent-b + 2*b/n*i
        point = [xVals[(i*n)+j], yVals[(i*n)+j], 20]

        myPrinter.Go_Point(point, delay=0.02)
        if j == 0:
            time.sleep(2)
        zVals[(i*n)+j] = mySensor.get_measurement()
    
myPrinter.Go_Home()
myPrinter.disconnect()
mySensor.disconnect_rs485()

print("Writing to File")
'''circle'''
# xVals = np.full(n, np.nan)
# yVals = np.full(n, np.nan)
# zVals = np.full(n, np.nan)
# for i in range(n):
#     xVals[i] = xCent+r*np.cos(theta[i])
#     yVals[i] = y=yCent+r*np.sin(theta[i])
#     zVals[i] = vals[i]

hubFacePoints = np.array([xVals, yVals, zVals])
np.savetxt('measuredData.txt', hubFacePoints.T, fmt='%.6f', delimiter=' ', header='X Y Z', comments='')
print("Plotting Data")
plt.scatter(x, vals, s=5)
plt.show()
