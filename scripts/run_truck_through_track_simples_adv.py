# -*- coding: utf-8 -*-
"""
Created on Fri Jun  9 16:20:37 2017

@author: leo
"""


from bike_trailer_model import BikeTrailer
from velocity_profile import RoadLimits
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
import math
from math import cos, sin

N = 600
dt = 0.01
speedLimit = [20 for i in range(0,N)]
angles = [-math.pi/4 if i < N/2 - 1 else math.pi/4  for i in range(0,N)]
x = [0]
y = [0]
for i in range(1,N):
    x.append(speedLimit[i]*dt*cos(angles[i]) + x[i-1])
    y.append(speedLimit[i]*dt*sin(angles[i]) + y[i-1])

truck = BikeTrailer(theta = -math.pi/6, x_0 = x[0], y_0 = y[0], lengthTractor = 4, lengthTrailer = 10, tailDistance = 0.5)
truck.driver.initPhiPIDController(K_p = 8, K_i = 0.4, K_d = 0)
#truck.driver.initPhiPIDController(K_p = .2, K_i = 0.0, K_d = 0)

fig = plt.figure(figsize=(6, 8))
ax = fig.add_subplot(2, 1, 1)
axes = [ax]

# The bottom independent axes
axes.append(fig.add_subplot(2, 1, 2))

ax =  axes[0]
ax2 = axes[1]

ax.scatter(x, y,c=speedLimit, marker='*', s=4)
ax2.scatter(x,y,c=speedLimit, marker='*', s=4)

#cax = fig.add_axes([0.9, 0.2, 0.02, 0.5])
#fig.colorbar(plot_xy, cax=cax, orientation='vertical', label = 'Velocity (m/s)')
plt.show()
#    plot_xy = ax2.scatter(centerArray[0],centerArray[1],c=range(len(kappa)), marker='*', s=4)

#ax2.set_xlim(55e4, 59e4)
#ax2.set_ylim(413e4, 416e4)

truck.createPlot(fig, ax)
truck.tractor.cruiseControl.v = speedLimit[0]
truck.tractor.cruiseControl.phi = 0

#interpV = interpolate.LinearNDInterpolator(list(zip(x, y)), speedLimit)
truck.driver.setRoadTracker(x, y, angles, speedLimit)

totalTime = 10
t = 0
truck.plotRefreshRate = .01
#
while t < totalTime:
    
    v = speedLimit[0]
#    omega = truck.calculateTracker(dt)
    omega = truck.calculateAdvancedTracker(dt)
#    phi = truck.cteLoop(dt)


    truck.tractor.cruiseControl.setOmega(omega)
#    truck.tractor.cruiseControl.setPhiRef(phi)
    truck.tractor.cruiseControl.setVelocityRef(v)
    
    truck.move(dt)
    if( t - truck.timeOfLastPlot >= truck.plotRefreshRate):   
#        print(phi)
        truck.plot()
        ax2.plot(truck.tractor.x, truck.tractor.y,'k*')
        truck.timeOfLastPlot = t

    t += dt
