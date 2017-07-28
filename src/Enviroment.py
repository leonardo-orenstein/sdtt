# -*- coding: utf-8 -*-
"""
Created on Thu Jul 13 15:54:09 2017

@author: leo
"""

import matplotlib.pyplot as plt
from matplotlib.patches import Arrow
from matplotlib.patches import Circle

from Body import Body
from LIDAR import LIDAR
from math import sin, cos, tan, pi, exp, sqrt
import numpy as np
import timeit

class SimulationEnviroment(object):
    '''
    Creates a map of the vehicle and its surroding enviroment. This map is used
    by the planner to avoid collision.

    This might be expanded later with a SLAM.
    '''

    def __init__(self, centerBody, mapRadius = 80):
        self.bodies = []
        self.mapRadius = mapRadius
        self.centerBody = centerBody
        self.centerBodyLine  = None
        self.centerBodyArrow = None
        self.radiusCircle = None
        self.bodyLines  = [] # might be better changin g this to a dict
        self.bodyArrows = [] # as to avoid having to keep bodies on track
        self.lidarArray = []

    def addLIDAR(self, lidar):
        self.lidarArray.append(lidar)

    def getPointMap(self):
        pm = []
        for lidar in self.lidarArray:
            pm.extend(lidar.pointMap)

        return pm

    def lidarScan(self):
        for lidar in self.lidarArray:
            lidar.updateRays()
            lidar.rayTracing(self.bodies)

    def addBody(self, body):
        if(body not in self.bodies):
            self.bodies.append(body)

    def update(self, t):
        for b in self.bodies:
            b.updateStates(t)

#        prevPos = self.centerBody.getPosition()
#        prevOrientation = self.centerBody.getOrientation()
#        self.centerBody.updateStates(t)
        pos = (self.centerBody.x, self.centerBody.y)
#        delta_x, delta_y = (currPos[0] - prevPos[0], currPos[1] - prevPos[1])
#        delta_theta = self.centerBody.getOrientation() - prevOrientation
        for lidar in self.lidarArray:
            lidar.setPose(pos, 0)

        self.lidarScan()

#        self.centerBody.update(t)

    def createPlot(self, fig = None, ax = None):
        self.timeOfLastPlot = -1

        if(fig is None):
            self.fig = plt.figure()
        else:
            self.fig = fig

        if(ax is None):
            self.ax = self.fig.add_subplot(111)
        else:
            self.ax = ax

        for b in self.bodies:
            (verticesX, verticesY) = b.getDrawingVertex()
            bodyLine, = self.ax.plot(verticesX, verticesY, 'k')
            self.bodyLines.append(bodyLine)
            directionArrow = Arrow(b.x, b.y,
                                         0.1*b.v*cos(b.orientation),
                                         0.1*b.v*sin(b.orientation),
                                         color = 'c')
            self.bodyArrows.append(directionArrow)
            self.ax.add_patch(directionArrow)

        (verticesX, verticesY) = self.centerBody.getDrawingVertex()
        self.centerBodyLine, = self.ax.plot(verticesX, verticesY, 'r')
        self.centerBodyArrow = Arrow(self.centerBody.x, self.centerBody.y,
                                     0.1*b.v*cos(self.centerBody.orientation),
                                     0.1*b.v*sin(self.centerBody.orientation),
                                     color = 'c')
        self.ax.add_patch(self.centerBodyArrow)

        self.radiusCircle = Circle((self.centerBody.x, self.centerBody.y),
                                     self.mapRadius,
                                     color = 'k',
                                     linestyle = ':',
                                     fill=False)
        self.ax.add_patch(self.radiusCircle)

        self.ax.set_xlim([self.centerBody.x - self.mapRadius*1.1, self.centerBody.x + self.mapRadius*1.1])
        self.ax.set_ylim([self.centerBody.y - self.mapRadius*1.1, self.centerBody.y + self.mapRadius*1.1])
        self.ax.set_xlabel('Distance X')
        self.ax.set_ylabel('Distance Y')


    def plot(self, draw = True):
        for idx in range(len(self.bodies)):
            b = self.bodies[idx]
            (verticesX, verticesY) = b.getDrawingVertex()
            bodyLine = self.bodyLines[idx]
            bodyLine.set_ydata(verticesY)
            bodyLine.set_xdata(verticesX)

            directionArrow = self.bodyArrows[idx]
            directionArrow.remove()

            directionArrow = Arrow(b.x, b.y,
                                         0.1*b.v*cos(b.orientation),
                                         0.1*b.v*sin(b.orientation),
                                         color = 'c')
            self.ax.add_patch(directionArrow)
            self.bodyArrows[idx] = directionArrow

        (verticesX, verticesY) = self.centerBody.getDrawingVertex()
        self.centerBodyLine.set_ydata(verticesY)
        self.centerBodyLine.set_xdata(verticesX)

        self.centerBodyArrow.remove()
        self.centerBodyArrow = Arrow(self.centerBody.x, self.centerBody.y,
                                     0.1*self.centerBody.v*cos(self.centerBody.orientation),
                                     0.1*self.centerBody.v*sin(self.centerBody.orientation),
                                     color = 'c')
        self.ax.add_patch(self.centerBodyArrow)

        self.radiusCircle.remove()
        self.radiusCircle = Circle((self.centerBody.x, self.centerBody.y),
                                     self.mapRadius,
                                     color = 'k',
                                     linestyle = ':',
                                     fill=False)
        self.ax.add_patch(self.radiusCircle)

        self.ax.set_xlim([self.centerBody.x - self.mapRadius*1.1, self.centerBody.x + self.mapRadius*1.1])
        self.ax.set_ylim([self.centerBody.y - self.mapRadius*1.1, self.centerBody.y + self.mapRadius*1.1])


        if(draw):
            self.fig.canvas.draw()
            plt.pause(0.001)

class Enviroment(SimulationEnviroment):
    '''
    Creates a map of the vehicle and its surroding enviroment. This map is used
    by the planner to avoid collision.

    This might be expanded later with a SLAM.
    '''

    def __init__(self, vehicle, mapRadius = 80):
        self.bodies = []
        self.mapRadius = mapRadius
        self.centerBody = Body.fromVehicle(vehicle)
        self.centerBodyLine  = None
        self.centerBodyArrow = None
        self.vehicle = vehicle
        self.radiusCircle = None
        self.bodyLines  = [] # might be better changing this to a dict
        self.bodyArrows = [] # as to avoid having to keep bodies on track

    def addBody(self, body):
        if(body not in self.bodies and self.centerBody.getDistance(body) <= self.mapRadius):
            self.bodies.append(body)

    def lidarScan(self):
        for lidar in self.vehicle.lidarArray:
            lidar.updateRays()
            lidar.rayTracing(self.bodies)

    def update(self, t):
        bodiesToRemove = []
        for idx in range(len(self.bodies)):
            body = self.bodies[idx]
            body.updateStates(t)
            if(self.centerBody.getDistance(body) > self.mapRadius):
                bodiesToRemove.append(idx)

#        for removeIdx in bodiesToRemove:
#            del self.bodies[removeIdx]

        orientation = self.vehicle.getOrientation()
        pos = self.vehicle.getPos()
        vel = self.vehicle.getVelocity()
        acc = self.vehicle.getAcc()
        omega = self.vehicle.getOmega()

        self.centerBody.setStates(pos[0], pos[1], vel, orientation, t)
        self.centerBody.setAcceleration(acc)
        self.centerBody.setOmega(omega)

    def createPlot(self, fig = None, ax = None):
        self.timeOfLastPlot = -1

        if(fig is None):
            self.fig = plt.figure()
        else:
            self.fig = fig

        if(ax is None):
            self.ax = self.fig.add_subplot(111)
        else:
            self.ax = ax

#        for lidar in self.vehicle.lidarArray:
        pm = [pm.coords if pm is not None else None for pm in self.vehicle.lidar.read()]
        bodyLine, = self.ax.plot(pm, 'r-', linewidth=0.1)
        self.bodyLines.append(bodyLine)

        (verticesX, verticesY) = self.centerBody.getDrawingVertex()
        self.centerBodyLine, = self.ax.plot(verticesX, verticesY, 'r')
        self.centerBodyArrow = Arrow(self.centerBody.x, self.centerBody.y,
                                     0.1*self.centerBody.v*cos(self.centerBody.orientation),
                                     0.1*self.centerBody.v*sin(self.centerBody.orientation),
                                     color = 'c')
        self.ax.add_patch(self.centerBodyArrow)

        self.radiusCircle = Circle((self.centerBody.x, self.centerBody.y),
                                     self.mapRadius,
                                     color = 'k',
                                     linestyle = ':',
                                     fill=False)
        self.ax.add_patch(self.radiusCircle)

        self.ax.set_xlim([self.centerBody.x - self.mapRadius*1.1, self.centerBody.x + self.mapRadius*1.1])
        self.ax.set_ylim([self.centerBody.y - self.mapRadius*1.1, self.centerBody.y + self.mapRadius*1.1])
        self.ax.set_xlabel('Distance X')
        self.ax.set_ylabel('Distance Y')


    def plot(self, draw = True):
#        for idx in range(len(self.vehicle.lidarArray)):

        idx = 0
        lidar = self.vehicle.lidar
        pm_x = [(self.centerBody.x, pm.xy[0][0]) if pm is not None else None for pm in lidar.read()]
        pm_y = [(self.centerBody.y, pm.xy[1][0]) if pm is not None else None for pm in lidar.read()]

        bodyLine = self.bodyLines[idx]
        bodyLine.set_ydata(pm_y)
        bodyLine.set_xdata(pm_x)

        (verticesX, verticesY) = self.centerBody.getDrawingVertex()
        self.centerBodyLine.set_ydata(verticesY)
        self.centerBodyLine.set_xdata(verticesX)

        self.centerBodyArrow.remove()
        self.centerBodyArrow = Arrow(self.centerBody.x, self.centerBody.y,
                                     0.1*self.centerBody.v*cos(self.centerBody.orientation),
                                     0.1*self.centerBody.v*sin(self.centerBody.orientation),
                                     color = 'c')
        self.ax.add_patch(self.centerBodyArrow)

        self.radiusCircle.remove()
        self.radiusCircle = Circle((self.centerBody.x, self.centerBody.y),
                                     self.mapRadius,
                                     color = 'k',
                                     linestyle = ':',
                                     fill=False)
        self.ax.add_patch(self.radiusCircle)

        self.ax.set_xlim([self.centerBody.x - self.mapRadius*1.1, self.centerBody.x + self.mapRadius*1.1])
        self.ax.set_ylim([self.centerBody.y - self.mapRadius*1.1, self.centerBody.y + self.mapRadius*1.1])


        if(draw):
            self.fig.canvas.draw()
            plt.pause(0.001)