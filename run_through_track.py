# -*- coding: utf-8 -*-
"""
Created on Fri Jun  9 16:20:37 2017

@author: leo
"""
from Vehicle import Vehicle
from SimulationVehicle import BikeTrailer
#from vpython import *

import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
import math
from math import cos, sin

# Create Track
N = 600
dt = 0.01
speedLimit = [i/20 if i > N/2 else 5 + (N/2)/20 - i/20 for i in range(0,N)]
#speedLimit = [20 for i in range(0,N)]

angles = [-math.pi/4 if i < N/2 - 1 else math.pi/4  for i in range(0,N)]

x = [0]
y = [0]
for i in range(1,N):
    x.append(speedLimit[i]*dt*cos(angles[i]) + x[i-1])
    y.append(speedLimit[i]*dt*sin(angles[i]) + y[i-1])

# Create plots to show the animation (top) and position on track (bottom)
fig = plt.figure(figsize=(6, 8))
axes = [fig.add_subplot(3, 1, 1)]

# The bottom independent axes
axes.append(fig.add_subplot(3, 1, 2))
axes.append(fig.add_subplot(3, 1, 3))

ax_simulation =  axes[0]
ax_vehicle = axes[1]
ax_path = axes[2]

ax_simulation.scatter(x, y,c=speedLimit, marker='*', s=4)
ax_vehicle.scatter(x,y,c=speedLimit, marker='*', s=4)
ax_path.scatter(x,y,c=speedLimit, marker='*', s=4)

plt.show()

# Create virtual vehicle (simulation)
truck = BikeTrailer(theta = -math.pi/6, x_0 = x[0], y_0 = y[0], lengthTractor = 4, lengthTrailer = 10, tailDistance = 0.5)
truck.createPlot(fig, ax_simulation)
truck.tractor.v = speedLimit[0]
truck.tractor.phi = 0
# Refresh rate of the simulation
truck.plotRefreshRate = .1

# Create the top layer of the autonomous vehicle
sdv  = Vehicle(length = 4, fig = fig, ax = ax_vehicle)

# Initializes the controller
sdv.pilot.initPhiPIDController(K_p = 8, K_i = 0.4, K_d = 1)

# Set the track to follow. We are tracking a gps reference here
sdv.planner.setTrack(x, y, angles, speedLimit)

# connect the autonomous vehcile system to the 'physical' system
sdv.connectToSimulation(truck.tractor)

# Total time of the simulation
totalTime = 60
t = 0

#body3d = box(pos=vector(0,0,0), axis=vector(0, 0, 0), length=length, height=2, width=2)

# simulation loop
while t < totalTime:
    
    sdv.scan()
    
    omega = sdv.headingTracker(dt)
#    omega = sdv.phiController(dt)
#    omega = sdv.cteController(dt)

    acc = sdv.velocityController(dt)
    
    sdv.engine.setValue(acc)
    sdv.steering.setValue(omega)
    
    truck.move(dt)
    if( t - truck.timeOfLastPlot >= truck.plotRefreshRate):   
#        print(phi)
        truck.plot()
        sdv.plot()
        x,y = sdv.gps.read()
        orientation = sdv.compass.read()
#        body3d.pos  = vector(x,y,0)
#        body3d.axis = vector(cos(orientation), sin(orientation), 0)
        ax_path.plot(truck.tractor.x, truck.tractor.y,'k*')
        truck.timeOfLastPlot = t

    t += dt

