# -*- coding: utf-8 -*-
"""
Created on Fri Jun  9 16:20:37 2017

@author: leo
"""
from Vehicle import Vehicle
from SimulationVehicle import BikeTrailer
import Track

#from vpython import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation, rc

from scipy import interpolate
import math
from math import cos, sin, pi
import random

random.seed(123)
np.random.seed(123)

# Create Track

dt = 0.01

a = -pi/4
ss1 = Track.StraightSegment([i/20 if i > 100 else 12 - i/20 for i in range(400)] , a)
acs = Track.ArcCurveSegment([20 for _ in range(40)], 10, a, dt)
ss2 = Track.StraightSegment([2 + i/20 if i < 200 else 12 for i in range(400)] , acs.angles[-1])

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

#speedLimit = [i/20 if i > N/2 else 2 + (N/2)/20 - i/20 for i in range(0,N)]
##speedLimit = [20 for i in range(0,N)]
#
#angles = [-pi/4 if i < N/2 - 1 else pi/4  for i in range(0,N)]
#
#x = [0]
#y = [0]
#roadWidth = 7
#for i in range(1,N):
#    x.append(speedLimit[i]*dt*cos(angles[i]) + x[i-1])
#    y.append(speedLimit[i]*dt*sin(angles[i]) + y[i-1])
#
#xUpper = []
#xLower = []
#yUpper = []
#yLower = []
#xCenterLane = []
#yCenterLane = []
#for i in range(0,N):
#    xCenterLane.append(x[i] + sin(-angles[i])*roadWidth/4)
#    yCenterLane.append(y[i] + cos(-angles[i])*roadWidth/4)
#    xUpper.append(x[i] + sin(-angles[i])*(roadWidth - roadWidth/4))
#    xLower.append(x[i] - sin(-angles[i])*roadWidth/4)
#    yUpper.append(y[i] + cos(-angles[i])*(roadWidth - roadWidth/4))
#    yLower.append(y[i] - cos(-angles[i])*roadWidth/4)

# Create plots to show the animation (top) and position on track (bottom)
fig = plt.figure(figsize=(6, 8))
axes = [fig.add_subplot(3, 1, 1)]

# The bottom independent axes
axes.append(fig.add_subplot(3, 1, 2))
axes.append(fig.add_subplot(3, 1, 3))

ax_simulation =  axes[0]
ax_vehicle = axes[1]
ax_path = axes[2]

ax_simulation.plot(xUpper, yUpper, 'k-')
ax_simulation.plot(xLower, yLower, 'k-')
ax_simulation.fill_between(xLower, yLower,yUpper, facecolor='black', interpolate = False)
ax_simulation.fill_between(xUpper, yLower,yUpper, facecolor='black', interpolate = False)
ax_simulation.plot(xCenterLane, yCenterLane,'w--')
ax_simulation.scatter(x,y,c=speedLimit, marker='*', s=4)

#ax_simulation.scatter(x, y,c=speedLimit, marker='*', s=4)
ax_vehicle.scatter(x,y,c=speedLimit, marker='*', s=4)

ax_path.scatter(x,y,c=speedLimit, marker='*', s=4)
ax_path.fill_between(xLower, yLower,yUpper, facecolor='black', interpolate = False)
ax_path.fill_between(xUpper, yLower,yUpper, facecolor='black', interpolate = False)
ax_path.plot(xCenterLane, yCenterLane,'w--')
ax_path.plot(xUpper, yUpper, 'k-')
ax_path.plot(xLower, yLower, 'k-')

ax_simulation.set_title('Simulation')
ax_vehicle.set_title('Measured data')
ax_path.set_title('Path taken')
plt.tight_layout(pad=0.4, w_pad=0.5, h_pad=2.0)

#ax_simulation.scatter(x, y, marker='*', s=4)
#ax_vehicle.scatter(x,y, marker='*', s=4)
#ax_path.scatter(x,y,marker='*', s=4)

time_text = ax_simulation.text(0.02, 0.95, '', transform=ax_simulation.transAxes)

#plt.show()

# Create virtual vehicle (simulation)
truck = BikeTrailer(theta = -pi/6, x_0 = x[0], y_0 = y[0], lengthTractor = 4, lengthTrailer = 10, tailDistance = 0.5)
truck.setNoise(errorPos = 2, errorPhi = 2*pi/180, errorOrientation = 5*pi/180, errorVelocity = 5)

truck.createPlot(fig, ax_simulation)
truck.tractor.v = 0
truck.tractor.phi = 0
# Refresh rate of the simulation
truck.plotRefreshRate = .1

# Create the top layer of the autonomous vehicle
sdv  = Vehicle(length = 4, fig = fig, ax = ax_vehicle)

# Initializes the controller
sdv.pilot.initPhiPIDController(K_p = 32, K_i = 0.4, K_d = 1)

# Set the track to follow. We are tracking a gps reference here
sdv.planner.setTrack(x, y, angles, speedLimit)

# connect the autonomous vehcile system to the 'physical' system
sdv.connectToSimulation(truck.tractor)

# First scan to get an initial position
sdv.scan()

# create the particle filter
sdv.createFilter()
# starts the inputs at 0
sdv.engine.setValue(0)
sdv.steering.setValue(0)


#body3d = box(pos=vector(0,0,0), axis=vector(0, 0, 0), length=length, height=2, width=2)

# simulation loop

# starts the system and acquire first positions
for _ in range(int(1/dt)):
    sdv.scan()
    sdv.updateFilter(dt)
    truck.move(dt)
    
## Total time of the simulation
#totalTime = 8
#t = 0  
#RSE = 0  
#while t < totalTime:
#    
#    sdv.scan()
#    sdv.updateFilter(dt)
#
#    omega = sdv.headingTracker(dt)
##    omega = sdv.phiController(dt)
##    omega = sdv.cteController(dt)
#
#    acc = sdv.velocityController(dt)
#    
#    sdv.engine.setValue(acc)
#    sdv.steering.setValue(omega)
#    RSE += ((sdv.planner.nextX - truck.tractor.x)**2 + (sdv.planner.nextY - truck.tractor.y)**2)**0.5
#    truck.move(dt)
#    if( t - truck.timeOfLastPlot >= truck.plotRefreshRate):   
##        print(phi)
#        truck.plot()
#        sdv.plot()
##        xHat,yHat = sdv.gps.read()
##        orientationHat = sdv.compass.read()
##        body3d.pos  = vector(x,y,0)
##        body3d.axis = vector(cos(orientation), sin(orientation), 0)
#        ax_path.plot(truck.tractor.x, truck.tractor.y,'y*')
#            
#        truck.timeOfLastPlot = t
#
#    t += dt
#print('MRSE:')
#print(RSE/(totalTime/dt))

linePath = None
def init():
    global truck, fig, ax, ax2, linePath
    
    ax_simulation.plot(xUpper, yUpper, 'k-')
    ax_simulation.plot(xLower, yLower, 'k-')
    ax_simulation.fill_between(xLower, yLower,yUpper, facecolor='black', interpolate = False)
    ax_simulation.fill_between(xUpper, yLower,yUpper, facecolor='black', interpolate = False)
    ax_simulation.plot(xCenterLane, yCenterLane,'w--')
    ax_simulation.scatter(x,y,c=speedLimit, marker='*', s=4)
    
    ax_path.scatter(x,y,c=speedLimit, marker='*', s=4)
    ax_path.fill_between(xLower, yLower,yUpper, facecolor='black', interpolate = False)
    ax_path.fill_between(xUpper, yLower,yUpper, facecolor='black', interpolate = False)
    ax_path.plot(xCenterLane, yCenterLane,'w--')
    ax_path.plot(xUpper, yUpper, 'k-')
    ax_path.plot(xLower, yLower, 'k-')
    
    linePath, = ax_path.plot(truck.tractor.x,truck.tractor.y, 'y*')

    time_text.set_text('')

    return (truck.tractorLine, sdv.vizualizer.body, linePath, time_text)

def animate(f):
    global truck, sdv, dt, linePath

    sdv.scan()
    sdv.updateFilter(dt)

    omega = sdv.headingTracker(dt)
    acc = sdv.velocityController(dt)
    
    sdv.engine.setValue(acc)
    sdv.steering.setValue(omega)
    
    truck.move(dt)
#    if(f % 10 == 0):
    truck.plot(False)
    sdv.plot(False)

    linePath.set_xdata(np.append(linePath.get_xdata(), truck.tractor.x))
    linePath.set_ydata(np.append(linePath.get_ydata(), truck.tractor.y))
    time_text.set_text('time = %.1f' % truck.timeOfLastPlot )

    return (truck.tractorLine, sdv.vizualizer.body, linePath, time_text)


#while t < totalTime:
# call the animator. blit=True means only re-draw the parts that have changed.
anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=800, interval=5, save_count = 50)    
#
### Set up formatting for the movie files
#Writer = animation.writers['ffmpeg']
#writer = Writer(fps=15, metadata=dict(artist='leo.ore'), bitrate=1800)

anim.save('truck.gif', writer = 'imagemagick')
