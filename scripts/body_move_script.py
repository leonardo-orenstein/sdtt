# -*- coding: utf-8 -*-
"""
Created on Mon Jul 24 15:41:22 2017

@author: leo
"""

from Body import Body
from Enviroment import Enviroment
from Vehicle import Vehicle
from SimulationVehicle import BikeTrailer
import Track

import numpy as np
import matplotlib.pyplot as plt

from math import pi
import timeit

np.random.seed(123)

# Create Track

dt = 0.01

a = -pi/4
ss1 = Track.StraightSegment([2.0 + i/20.0 if i > 100.0 else 12.0 - i/20.0 for i in range(400)] , a)
acs = Track.ArcCurveSegment([22.0 for _ in range(40)], 10, a, dt)
ss2 = Track.StraightSegment([22.0 - i/20.0 if i < 200.0 else 12.0 for i in range(400)] , acs.angles[-1])

track = Track.Track([ss1,acs,ss2], dt)
N = track.N

speedLimit = track.vel
angles = track.angles

x = track.x
y = track.y

xUpper = track.xUpper
xLower = track.xLower
yUpper = track.yUpper
yLower = track.yLower
xCenterLane = track.xCenterLane
yCenterLane = track.yCenterLane

# Create plots to show the animation (top) and position on track (bottom)
fig = plt.figure(figsize=(6, 8))
axes = [fig.add_subplot(3, 1, 1)]

# The bottom independent axes
axes.append(fig.add_subplot(3, 2, 3))
axes.append(fig.add_subplot(3, 2, 4))
axes.append(fig.add_subplot(3, 1, 3))

ax_simulation =  axes[0]
ax_vehicle = axes[1]
ax_env = axes[2]
ax_path = axes[3]

ax_simulation.plot(xUpper, yUpper, 'k-')
ax_simulation.plot(xLower, yLower, 'k-')
ax_simulation.fill_between(xLower, yLower,yUpper, facecolor='black', interpolate = False)
ax_simulation.fill_between(xUpper, yLower,yUpper, facecolor='black', interpolate = False)
ax_simulation.plot(xCenterLane, yCenterLane,'w--')
ax_simulation.scatter(x,y,c=speedLimit, marker='*', s=4)

ax_vehicle.scatter(x,y,c=speedLimit, marker='*', s=4)

ax_path.scatter(x,y,c=speedLimit, marker='*', s=4)
ax_path.fill_between(xLower, yLower,yUpper, facecolor='black', interpolate = False)
ax_path.fill_between(xUpper, yLower,yUpper, facecolor='black', interpolate = False)
ax_path.plot(xCenterLane, yCenterLane,'w--')
ax_path.plot(xUpper, yUpper, 'k-')
ax_path.plot(xLower, yLower, 'k-')

ax_simulation.set_title('Simulation')
ax_vehicle.set_title('Measured data')
ax_env.set_title('Enviroment')
ax_path.set_title('Path taken')
plt.tight_layout(pad=0.4, w_pad=0.5, h_pad=2.0)

time_text = ax_simulation.text(0.02, 0.05, '', transform=ax_simulation.transAxes)

# Create virtual vehicle (simulation)
truck = BikeTrailer(theta = -pi/6.0, x_0 = x[0], y_0 = y[0], lengthTractor = 4.0, lengthTrailer = 10.0, tailDistance = 0.5)
truck.setNoise(errorPos = 2.0, errorPhi = 2.0*pi/180, errorOrientation = 5.0*pi/180, errorVelocity = 5.0)

truck.createPlot(fig, ax_simulation)
truck.tractor.v = 0
truck.tractor.phi = 0
# Refresh rate of the simulation
truck.plotRefreshRate = .05

# Create the top layer of the autonomous vehicle
sdv  = Vehicle(length = 4, fig = fig, ax = ax_vehicle)

# Initializes the controller
sdv.pilot.initPhiPIDController(K_p = 32.0, K_i = 0.4, K_d = 1.0)

# Set the track to follow. We are tracking a gps reference here
sdv.planner.setTrack(x, y, angles, speedLimit, False)

def laneTracker():
    global truck
    return truck.tractor.x, truck.tractor.y

# connect the autonomous vehcile system to the 'physical' system
sdv.connectToSimulation(truck.tractor, laneTracker)
sdv.compass.setUncertanty(2.0*pi/180.0)
sdv.wheelAngle.setUncertanty(1.0*pi/180.0)
sdv.speedometer.setUncertanty(5.0, True)
sdv.gps.setUncertanty(5.0)
sdv.laneTracker.setUncertanty(.1)

# First scan to get an initial position
sdv.scan()

# create the particle filter
sdv.createFilter()

# starts the inputs at 0
sdv.engine.setValue(0.0)
sdv.steering.setValue(0.0)

# creates an eviroment with 3 bodies
vertices = [(-2,-2), (2,-2), (1,1),(-1,1)]
x, y = sdv.getPos()
b1 = Body(orientation= 0, x = x-10, y = y, v = .1, acc = 0, omega = 0.1, vertex= vertices)
b2 = Body(orientation= pi/2, x = x+10, y = y+10, v = .2, acc = 0, omega = 0.1, vertex= vertices)
b3 = Body(orientation= pi/2, x = x+10, y = y-10, v = .1, acc = 0, omega = 0.1, vertex= vertices)

env = Enviroment(sdv)
env.addBody(b1)
env.addBody(b2)
env.addBody(b3)

env.createPlot(fig = fig, ax=ax_env)

# simulation loop
start = timeit.default_timer()

# starts the system and acquire first positions
for _ in range(int(1/dt)):
    sdv.scan()
    sdv.updateFilter(dt)
    truck.move(dt)
    time_text.set_text('Initializng systems' )
    truck.plot(False)
    sdv.plot(False)

# Total time of the simulation
totalTime = 9
t = 0
linePath, = ax_path.plot(truck.tractor.x, truck.tractor.y,'y*')
while t < totalTime:

    sdv.scan()

    sdv.updateFilter(dt)

    omega = sdv.headingTracker(dt)
    acc = sdv.velocityController(dt)

    sdv.engine.setValue(acc)
    sdv.steering.setValue(omega)

    truck.move(dt)
    env.updateStates(t)
    if( t - truck.timeOfLastPlot >= truck.plotRefreshRate):
        truck.plot()
        sdv.plot()
        env.plot()
        time_text.set_text('time = %.1f' % t )
        linePath.set_xdata(np.append(linePath.get_xdata(), truck.tractor.x))
        linePath.set_ydata(np.append(linePath.get_ydata(), truck.tractor.y))
        truck.timeOfLastPlot = t


    t += dt

stop = timeit.default_timer()
