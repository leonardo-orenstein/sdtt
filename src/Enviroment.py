# -*- coding: utf-8 -*-
"""
Created on Thu Jul 13 15:54:09 2017

@author: leo
"""

import matplotlib.pyplot as plt
from matplotlib.patches import Arrow
from matplotlib.patches import Circle

from Body import Body
from math import sin, cos, tan, pi, exp, sqrt
import numpy as np

class Enviroment(object):
    '''
    Creates a map of the vehicle and its surroding enviroment. This map is used
    by the planner to avoid collision.

    This might be expanded later with a SLAM.
    '''

    def __init__(self, vehicle, mapRadius = 80):
        self.bodies = []
        self.mapRadius = mapRadius
        self.vehicleBody = Body.fromVehicle(vehicle)
        self.vehicle = vehicle
        self.vehicleLine  = None
        self.vehicleArrow = None
        self.radiusCircle = None
        self.bodyLines  = [] # might be better changing this to a dict
        self.bodyArrows = [] # as to avoid having to keep bodies on track

    def addBody(self, body):
        if(body not in self.bodies and self.vehicleBody.getDistance(body) <= self.mapRadius):
            self.bodies.append(body)

    def updateEnviroment(self, t):
        self.vehicleBody.updateStates(t)
        updatedBodies = []
        for b in self.bodies:
            b.updateStates(t)
            if(self.vehicleBody.getDistance(b) <= self.mapRadius):
                updatedBodies.append(b)
        self.bodies = updatedBodies
        self.vehicleBody.updateStatesFromVehicle(self.vehicle, t)

    def updateStates(self, t):
        bodiesToRemove = []
        for idx in range(len(self.bodies)):
            body = self.bodies[idx]
            body.updateStates(t)
            if(self.vehicleBody.getDistance(body) > self.mapRadius):
                bodiesToRemove.append(idx)

        for removeIdx in bodiesToRemove:
            del self.bodies[removeIdx]
            self.bodyLines[removeIdx].remove()
            del self.bodyLines[removeIdx]
            self.bodyArrows[removeIdx].remove()
            del self.bodyArrows[removeIdx]

        orientation = self.vehicle.getOrientation()
        pos = self.vehicle.getPos()
        vel = self.vehicle.getVelocity()
        acc = self.vehicle.getAcc()
        omega = self.vehicle.getOmega()

        self.vehicleBody.setStates(pos[0], pos[1], vel, orientation, t)
        self.vehicleBody.setAcceleration(acc)
        self.vehicleBody.setOmega(omega)

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

        (verticesX, verticesY) = self.vehicleBody.getDrawingVertex()
        self.vehicleLine, = self.ax.plot(verticesX, verticesY, 'r')
        self.vehicleArrow = Arrow(self.vehicleBody.x, self.vehicleBody.y,
                                     0.1*b.v*cos(self.vehicleBody.orientation),
                                     0.1*b.v*sin(self.vehicleBody.orientation),
                                     color = 'c')
        self.ax.add_patch(self.vehicleArrow)

        self.radiusCircle = Circle((self.vehicleBody.x, self.vehicleBody.y),
                                     self.mapRadius,
                                     color = 'k',
                                     linestyle = ':',
                                     fill=False)
        self.ax.add_patch(self.radiusCircle)

        self.ax.set_xlim([self.vehicleBody.x - self.mapRadius*1.1, self.vehicleBody.x + self.mapRadius*1.1])
        self.ax.set_ylim([self.vehicleBody.y - self.mapRadius*1.1, self.vehicleBody.y + self.mapRadius*1.1])
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

            directionArrow = Arrow(self.vehicleBody.x, self.vehicleBody.y,
                                         0.1*b.v*cos(self.vehicleBody.orientation),
                                         0.1*b.v*sin(self.vehicleBody.orientation),
                                         color = 'c')
            self.ax.add_patch(directionArrow)
            self.bodyArrows[idx] = directionArrow

        (verticesX, verticesY) = self.vehicleBody.getDrawingVertex()
        self.vehicleLine.set_ydata(verticesY)
        self.vehicleLine.set_xdata(verticesX)

        self.vehicleArrow.remove()
        self.vehicleArrow = Arrow(self.vehicleBody.x, self.vehicleBody.y,
                                     0.1*self.vehicleBody.v*cos(self.vehicleBody.orientation),
                                     0.1*self.vehicleBody.v*sin(self.vehicleBody.orientation),
                                     color = 'c')

        self.radiusCircle.remove()
        self.radiusCircle = Circle((self.vehicleBody.x, self.vehicleBody.y),
                                     self.mapRadius,
                                     color = 'k',
                                     linestyle = ':',
                                     fill=False)
        self.ax.add_patch(self.radiusCircle)

        self.ax.add_patch(self.vehicleArrow)
        self.ax.set_xlim([self.vehicleBody.x - self.mapRadius*1.1, self.vehicleBody.x + self.mapRadius*1.1])
        self.ax.set_ylim([self.vehicleBody.y - self.mapRadius*1.1, self.vehicleBody.y + self.mapRadius*1.1])


        if(draw):
            self.fig.canvas.draw()
            plt.pause(0.001)