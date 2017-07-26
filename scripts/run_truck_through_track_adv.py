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
from math import pi

limits = RoadLimits()

start = 0
truck = BikeTrailer(theta = limits.angles[start], x_0 = limits.x[start], y_0 = limits.y[start], lengthTractor = 4, lengthTrailer = 10, tailDistance = 0.5)
truck.tractor.cruiseControl.setAcceleration(limits.max_lon_accel)
truck.driver.initPhiPIDController(K_p = 8, K_i = 0.4, K_d = 0)

fig = plt.figure(figsize=(6, 8))
ax = fig.add_subplot(2, 1, 1)
axes = [ax]

# The bottom independent axes
axes.append(fig.add_subplot(2, 1, 2))

ax =  axes[0]
ax2 = axes[1]

ax.plot(limits.x, limits.y, linewidth = 0.5, color = 'k')
ax.scatter(limits.x, limits.y,c=limits.speedLimit, marker='*',  s=8)

ax2.scatter(limits.x,limits.y,c=limits.speedLimit, marker='*', lineStyles = '-', s=4)


#cax = fig.add_axes([0.9, 0.2, 0.02, 0.5])
#fig.colorbar(plot_xy, cax=cax, orientation='vertical', label = 'Velocity (m/s)')
plt.show()
#    plot_xy = ax2.scatter(centerArray[0],centerArray[1],c=range(len(kappa)), marker='*', s=4)

ax2.set_xlim(55e4, 59e4)
ax2.set_ylim(413e4, 416e4)

truck.createPlot(fig, ax)
truck.tractor.cruiseControl.v = limits.speedLimit[start]
truck.tractor.cruiseControl.phi = 0

interpV = interpolate.NearestNDInterpolator (list(zip(limits.x, limits.y)), limits.speedLimit)
interpAngle = interpolate.NearestNDInterpolator (list(zip(limits.x, limits.y)), limits.angles)
truck.driver.setRoadTracker(limits.x, limits.y, limits.angles, limits.speedLimit)

dt = .1
totalTime = 3000
t = 0
truck.plotRefreshRate = .2
truck.driver.currGoal = start

states = []
x_1 = np.array([])
y_1 = np.array([])
delta_1 = np.array([])
delta_2 = np.array([])
v_tractor = np.array([])
acc_cent =  np.array([])
phi_array =  np.array([])
radius =  np.array([])

truck.plot()
truckLine, = ax2.plot(truck.tractor.x,truck.tractor.y,'k*')

while t < totalTime:
    
    v = interpV(truck.tractor.x, truck.tractor.y)
    omega = truck.calculateTracker(dt)
#    omega = truck.calculateAdvancedTracker(dt)

    truck.tractor.cruiseControl.setOmega(omega)
    if(truck.tractor.acc_cent > 0):
        v_cet = truck.centripetalLoop(limits.max_lat_accel)
        v = min(v, v_cet)
        
    truck.tractor.cruiseControl.setVelocityRef(v)
    
    X = truck.move(dt)
    x_1 = np.append(x_1, X[-1][0])
    y_1 = np.append(y_1, X[-1][1])
    delta_1 = np.append(delta_1, X[-1][2])
    delta_2 = np.append(delta_2, X[-1][3])
    v_tractor = np.append(v_tractor, truck.tractor.cruiseControl.v)
    phi_array = np.append(phi_array, truck.tractor.cruiseControl.phi)
    acc_cent = np.append(acc_cent, truck.tractor.acc_cent)
    radius = np.append(radius, truck.tractor.radius)
    
    
    states.append(X[-1])
    if( t - truck.timeOfLastPlot >= truck.plotRefreshRate):   
        truck.plot()
        truckLine.set_ydata(y_1)
        truckLine.set_xdata(x_1)
        ax2.plot(truck.tractor.x,truck.tractor.y,'k*')
        truck.timeOfLastPlot = t
    t += dt

truck.plot()
ax2.plot(truck.tractor.x,truck.tractor.y,'k*')
truck.timeOfLastPlot = t

#[x_1, y_1, delta_1, delta_2]
time = np.arange(0, totalTime+dt, dt)

states_fig, state_ax = plt.subplots(4, sharex=True)
state_ax[0].plot(time, x_1)
state_ax[0].set_title('States axis')
state_ax[0].set_ylabel('x_1')

state_ax[1].plot(time, y_1)
state_ax[1].set_ylabel('y_1')

state_ax[2].plot(time, delta_1)
state_ax[2].set_ylabel('delta_1')

state_ax[3].plot(time, delta_2)
state_ax[3].set_ylabel('delta_2')

velocity_fig = plt.figure()
velocity_ref_ax = velocity_fig.add_subplot(211)
velocity_actual_ax= velocity_fig.add_subplot(212)

velocity_actual_ax.scatter(x_1, y_1,c=v_tractor, cmap = 'plasma', marker='*', s=4)
velocity_actual_ax.set_title('Actual velocity')
velocity_ref_ax.scatter(limits.x,limits.y,c=limits.speedLimit, marker='*', s=4)
velocity_ref_ax.set_title('Reference velocity')

alpha_fig = plt.figure()
alpha_ax = alpha_fig.add_subplot(111)
alpha = (delta_2-delta_1) % (2*math.pi) 
alpha[alpha > pi] -= 2*pi
alpha_ax.scatter(x_1, y_1,c=delta_2-delta_1, cmap = 'PuOr', marker='*', s=4)
alpha_ax.set_title('Alpha')
# Still some oddities in alpha when orientation going from -pi / pi.
# Still not clear if is from the dynamics of the system, the controller or something in the implementation
# of the model. Nothing major, but I Have to get back to this.

acc_fig = plt.figure()
acc_ax = acc_fig.add_subplot(111)
acc_ax.scatter(x_1, y_1,c=acc_cent, cmap = 'PuOr', marker='*', s=4)
acc_ax.set_title('Centripetal Acceleration')

phi_fig = plt.figure()
phi_ax = phi_fig.add_subplot(111)
phi_ax.plot(time, phi_array)
phi_ax.set_ylabel('phi')
