# -*- coding: utf-8 -*-
"""
Created on Mon Jul 24 15:41:22 2017

@author: leo
"""

import sys
sys.path.append('../')

from src.Body import Body
from src.Enviroment import Enviroment, SimulationEnviroment
from src.Vehicle import Vehicle
from src.SimulationVehicle import BikeTrailer
from src import Track
from src.LIDAR import LIDAR

import numpy as np
import matplotlib.pyplot as plt

from math import pi
import timeit

np.random.seed(123)

# Create Track

dt = 0.01

a = -pi/4
ss1 = Track.StraightSegment([2.0 + i/20.0 if i > 100.0 else 12.0 - i/20.0 for i in range(400)] , a)
acs = Track.ArcCurveSegment([22.0 for _ in range(40)], 5, a, dt)
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
axes = [fig.add_subplot(4, 1, 1)]

# The bottom independent axes
axes.append(fig.add_subplot(4, 1, 2))
axes.append(fig.add_subplot(4, 2, 5))
axes.append(fig.add_subplot(4, 2, 6))
axes.append(fig.add_subplot(4, 1, 4))

ax_simulation =  axes[0]
ax_vehicle = axes[1]
ax_simul_env = axes[2]
ax_env = axes[3]
ax_path = axes[4]

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
ax_simul_env.set_title('Simul Enviroment')
ax_env.set_title('LIDAR')
ax_path.set_title('Path taken')
plt.tight_layout(pad=0.4, w_pad=0.5, h_pad=2.0)

time_text = ax_simulation.text(0.02, 0.05, '', transform=ax_simulation.transAxes)

# Create virtual vehicle (simulation)
truck = BikeTrailer(theta = -pi/6.0, x_0 = track.x[0], y_0 = track.y[0], lengthTractor = 4.0, lengthTrailer = 10.0, tailDistance = 0.5)
#truck.setNoise(errorPos = 2.0, errorPhi = 2.0*pi/180, errorOrientation = 5.0*pi/180, errorVelocity = 5.0)

truck.createPlot(fig, ax_simulation)
truck.tractor.v = 0
truck.tractor.phi = 0
# Refresh rate of the simulation
truck.plotRefreshRate = .05

# Create the top layer of the autonomous vehicle
sdv  = Vehicle(length = 4, fig = fig, ax = ax_vehicle)

# Initializes the controller
sdv.pilot.initPhiPIDController(K_p = 32.0, K_i = 0.4, K_d = 1.0)

# Set the track to follow. We are tracking an absolute position
sdv.planner.setTrack(x, y, angles, speedLimit, False)

def laneTracker():
    global truck
    return truck.tractor.x, truck.tractor.y

# connect the autonomous vehcile system to the 'physical' system
sdv.connectToSimulation(truck.tractor, laneTracker)
sdv.compass.setUncertanty(2.0*pi/180.0)
sdv.wheelAngle.setUncertanty(1.0*pi/180.0)
sdv.speedometer.setUncertanty(5.0, True)
sdv.gps.setUncertanty(1.0)
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
cbVertices = [(-2,-2), (2,-2), (2,2),(-2,2)]
x, y = truck.tractor.x, truck.tractor.y

b1 = Body(orientation= 0., x = x-10., y = y, v = .1, acc = 0., omega = 0.1, vertex= vertices)
b2 = Body(orientation= pi/2, x = x+10., y = y+10., v = .2, acc = 0., omega = 0.1, vertex= vertices)
b3 = Body(orientation= pi/4, x = x+10., y = y-10., v = .2, acc = 0., omega = 0.2, vertex= vertices)
b4 = Body(orientation= pi/2, x = x+np.random.rand()*10., y = y+np.random.rand()*10., v = np.random.rand(), acc = 0., omega = 0.1, vertex= vertices)
b5 = Body(orientation= pi/2, x = x+np.random.rand()*10., y = y-np.random.rand()*10., v = np.random.rand(), acc = 0., omega = 0.1, vertex= vertices)
b6 = Body(orientation= pi/2, x = x-np.random.rand()*10., y = y+np.random.rand()*10., v = np.random.rand(), acc = 0., omega = 0.1, vertex= vertices)
b7 = Body(orientation= pi/2, x = x-np.random.rand()*10., y = y-np.random.rand()*10., v = np.random.rand(), acc = 0., omega = 0.1, vertex= vertices)

lidar = LIDAR(truck.tractor.x, truck.tractor.y, 2., 80.)
sdv.addLIDAR(lidar)

simul_env = SimulationEnviroment(truck.tractor)
env = Enviroment(sdv)

simul_env.addBody(b1)
simul_env.addBody(b2)
simul_env.addBody(b3)
simul_env.addLIDAR(lidar)
#simul_env.addBody(b4)
#simul_env.addBody(b5)
#simul_env.addBody(b6)
#simul_env.addBody(b7)

env.createPlot(fig = fig, ax=ax_env)
simul_env.createPlot(fig = fig, ax=ax_simul_env)

#env.createPlot()
#simul_env.createPlot()

# simulation loop
start = timeit.default_timer()

# Total time of the simulation
totalTime = 9
t = 0
linePath, = ax_path.plot(truck.tractor.x, truck.tractor.y,'y*')
while t < totalTime:

    sdv.scan()
    sdv.lidarScan(simul_env)

#    sdv.updateFilter(dt)

    omega = sdv.headingTracker(dt)
#    omega  = sdv.lqrTracker(dt)

    acc = sdv.velocityController(dt)

    sdv.engine.setValue(acc)
    sdv.steering.setValue(omega)

    truck.move(dt)
    simul_env.update(t)
    env.update(t)

    if( t - truck.timeOfLastPlot >= truck.plotRefreshRate):

        env.plot(False)
        simul_env.plot(False)
        truck.plot(False)
        sdv.plot(True)
        time_text.set_text('time = %.1f' % t )
        linePath.set_xdata(np.append(linePath.get_xdata(), truck.tractor.x))
        linePath.set_ydata(np.append(linePath.get_ydata(), truck.tractor.y))
        truck.timeOfLastPlot = t


    t += dt

stop = timeit.default_timer()
